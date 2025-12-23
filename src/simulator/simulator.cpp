//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    simulator.cpp
\brief   C++ Implementation: Simulator
\author  Joydeep Biswas, (C) 2011
*/
//========================================================================

#include <math.h>
#include <memory>
#include <random>
#include <stdio.h>
#include <fstream>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "gflags/gflags.h"

#include "simulator.h"
#include "simulator/drive_models/ackermann_model.h"
#include "simulator/drive_models/omnidirectional_model.h"
#include "simulator/drive_models/diff_drive_model.h"
#include "simulator/drive_models/ideal_model.h"
#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "amrl_msgs/msg/localization2_d_msg.hpp"
#include <std_msgs/msg/string.hpp>
#include "vector_map.h"

using ackermann::AckermannModel;
using diffdrive::DiffDriveModel;
using Eigen::Rotation2Df;
using Eigen::Vector2f;
using geometry::Heading;
using geometry::Line2f;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using human::HumanObject;
using ideal::IdealModel;
using math_util::AngleMod;
using math_util::DegToRad;
using math_util::RadToDeg;
using omnidrive::OmnidirectionalModel;
using std::atan2;
using vector_map::VectorMap;

// Configuration values accessed via config_ member

Simulator::Simulator(const SimulatorConfig& config) : config_(config),
                                                      laser_noise_(0, 1),
                                                      sim_step_count(0),
                                                      sim_time(0.0),
                                                      current_map_name_(config.map_name) {
    if (config_.map_name == "") {
        std::cerr << "Failed to load map - map_name not specified in config" << std::endl;
        exit(1);
    }
}

Simulator::~Simulator() {}

robot_model::RobotModel* MakeMotionModel(const std::string& robot_type,
                                         const std::string& robot_config_file) {
    if (robot_type == "ACKERMANN_DRIVE") {
        return new AckermannModel(robot_config_file);
    } else if (robot_type == "OMNIDIRECTIONAL_DRIVE") {
        return new OmnidirectionalModel(robot_config_file);
    } else if (robot_type == "DIFF_DRIVE") {
        return new DiffDriveModel(robot_config_file);
    } else if (robot_type == "IDEAL_DRIVE") {
        return new IdealModel(robot_config_file);
    }
    std::cerr << "Robot type \"" << robot_type
              << "\" has no associated motion model!" << std::endl;
    return nullptr;
}

std::string IndexToPrefix(const size_t index) {
    return "robot" + std::to_string(index);
}

bool Simulator::init(rclcpp::Node::SharedPtr node) {
    // TODO(jaholtz) Too much hard coding, move to config
    node_ = node;

    scanDataMsg.header.frame_id = config_.laser_frame;
    scanDataMsg.angle_min = config_.laser_angle_min;
    scanDataMsg.angle_max = config_.laser_angle_max;
    scanDataMsg.angle_increment = config_.laser_angle_increment;
    scanDataMsg.range_min = config_.laser_min_range;
    scanDataMsg.range_max = config_.laser_max_range;
    scanDataMsg.intensities.clear();
    scanDataMsg.time_increment = 0.0;
    scanDataMsg.scan_time = 0.05;

    odometryTwistMsg.header.frame_id = "odom";
    odometryTwistMsg.child_frame_id = "base_link";

    const std::string map_path =
        config_.maps_dir + "/" + config_.map_name + "/" + config_.map_name + ".vectormap.txt";
    if (!std::ifstream(map_path.c_str()).good()) {
        std::cerr << "Failed to locate map file at \"" << map_path << "\". "
                  << "Set maps_dir or map_name correctly." << std::endl;
        return false;
    }
    map_.Load(map_path);

    // Create motion models for each robot
    for (size_t i = 0; i < config_.robots.size(); ++i) {
        const auto& robot = config_.robots[i];
        const auto pf = IndexToPrefix(i);
        auto* mm = MakeMotionModel(robot.type, robot.config_file);
        if (mm == nullptr) {
            return false;
        }
        mm->SetPose(Pose2Df(robot.start_pose.z(), {robot.start_pose.x(), robot.start_pose.y()}));

        // Initialize ROS interfaces for the motion model
        std::string drive_topic = "/cmd_vel";  // Standardized topic for all robots
        if (!mm->Init(node_, pf, drive_topic)) {
            return false;
        }

        // Load robot geometry from its config file
        CONFIG_FLOAT(car_length, "car_length");
        CONFIG_FLOAT(car_width, "car_width");
        CONFIG_VECTOR3F(laser_loc, "laser_loc");
        config_reader::ConfigReader robot_reader({robot.config_file});

        robot_pub_subs_.emplace_back(RobotPubSub());
        auto& rps = robot_pub_subs_.back();
        rps.car_length = CONFIG_car_length;
        rps.car_width = CONFIG_car_width;
        rps.laser_x = CONFIG_laser_loc.x();
        rps.laser_y = CONFIG_laser_loc.y();
        rps.laser_z = CONFIG_laser_loc.z();
        rps.motion_model = std::unique_ptr<robot_model::RobotModel>(mm);

        rps.initSubscriber = node_->create_subscription<amrl_msgs::msg::Localization2DMsg>(
            pf + "/initialpose", 1,
            [&rps](const amrl_msgs::msg::Localization2DMsg::SharedPtr msg) {
                const Vector2f loc(msg->pose.x, msg->pose.y);
                const float angle = msg->pose.theta;
                rps.motion_model->SetPose({angle, loc});
            });
        rps.odometryTwistPublisher = node_->create_publisher<nav_msgs::msg::Odometry>(pf + "/odom", 1);
        rps.laserPublisher = node_->create_publisher<sensor_msgs::msg::LaserScan>(pf + "/" + config_.laser_topic, 1);
        rps.localizationPublisher = node_->create_publisher<amrl_msgs::msg::Localization2DMsg>(
            pf + "/localization", 1);
        localizationMsg.header.frame_id = "map";
    }
    br = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
    // Subscribe to current map topic for dynamic map switching
    current_map_subscriber_ = node_->create_subscription<std_msgs::msg::String>(
        config_.current_map_topic, 1,
        std::bind(&Simulator::CurrentMapCallback, this, std::placeholders::_1));

    this->loadObject();
    return true;
}

// TODO(yifeng): Change this into a general way
void Simulator::loadObject() {
    for (const string& config_str : config_.short_term_object_configs) {
        objects.push_back(
            std::unique_ptr<ShortTermObject>(new ShortTermObject(config_str)));
    }

    for (const string& config_str : config_.human_configs) {
        objects.push_back(
            std::unique_ptr<HumanObject>(new HumanObject({config_str})));
    }
}

void Simulator::publishOdometry() {
    for (auto& rps : robot_pub_subs_) {
        tf2::Quaternion robotQ;
        robotQ.setRPY(0, 0, rps.cur_loc.angle);

        odometryTwistMsg.header.stamp = node_->now();
        odometryTwistMsg.pose.pose.position.x = rps.cur_loc.translation.x();
        odometryTwistMsg.pose.pose.position.y = rps.cur_loc.translation.y();
        odometryTwistMsg.pose.pose.position.z = 0.0;
        odometryTwistMsg.pose.pose.orientation.x = robotQ.x();
        odometryTwistMsg.pose.pose.orientation.y = robotQ.y();
        odometryTwistMsg.pose.pose.orientation.z = robotQ.z();
        odometryTwistMsg.pose.pose.orientation.w = robotQ.w();

        // High confidence in x, y position and yaw; no information about z, roll, pitch
        odometryTwistMsg.pose.covariance[0] = 0.00001;           // x position
        odometryTwistMsg.pose.covariance[7] = 0.00001;           // y position
        odometryTwistMsg.pose.covariance[14] = 1000000000000.0;  // z position (unknown)
        odometryTwistMsg.pose.covariance[21] = 1000000000000.0;  // roll (unknown)
        odometryTwistMsg.pose.covariance[28] = 1000000000000.0;  // pitch (unknown)
        odometryTwistMsg.pose.covariance[35] = 0.001;            // yaw

        // Since this is simulated odometry with perfect velocity knowledge
        odometryTwistMsg.twist.covariance[0] = 0.00001;           // linear x velocity
        odometryTwistMsg.twist.covariance[7] = 0.00001;           // linear y velocity
        odometryTwistMsg.twist.covariance[14] = 1000000000000.0;  // linear z velocity (unknown)
        odometryTwistMsg.twist.covariance[21] = 1000000000000.0;  // angular x velocity (unknown)
        odometryTwistMsg.twist.covariance[28] = 1000000000000.0;  // angular y velocity (unknown)
        odometryTwistMsg.twist.covariance[35] = 0.00001;          // angular z velocity

        odometryTwistMsg.twist.twist.angular.x = 0.0;
        odometryTwistMsg.twist.twist.angular.y = 0.0;
        odometryTwistMsg.twist.twist.angular.z = rps.vel.angle;
        odometryTwistMsg.twist.twist.linear.x = rps.vel.translation.x();
        odometryTwistMsg.twist.twist.linear.y = rps.vel.translation.y();
        odometryTwistMsg.twist.twist.linear.z = 0.0;

        rps.odometryTwistPublisher->publish(odometryTwistMsg);
    }
}

void Simulator::publishLaser() {
    for (size_t i = 0; i < robot_pub_subs_.size(); ++i) {
        auto& rps = robot_pub_subs_[i];
        scanDataMsg.header.stamp = node_->now();
        scanDataMsg.header.frame_id = IndexToPrefix(i) + config_.laser_frame;
        const Vector2f laserRobotLoc(rps.laser_x, rps.laser_y);
        const Vector2f laserLoc =
            rps.cur_loc.translation + Rotation2Df(rps.cur_loc.angle) * laserRobotLoc;

        const int num_rays = static_cast<int>(
            1.0 + (scanDataMsg.angle_max - scanDataMsg.angle_min) /
                      scanDataMsg.angle_increment);
        map_.GetPredictedScan(laserLoc,
                              scanDataMsg.range_min,
                              scanDataMsg.range_max,
                              scanDataMsg.angle_min + rps.cur_loc.angle,
                              scanDataMsg.angle_max + rps.cur_loc.angle,
                              num_rays,
                              &scanDataMsg.ranges);
        for (float& r : scanDataMsg.ranges) {
            if (r > scanDataMsg.range_max - 0.1) {
                r = 0;
                continue;
            }
            r = max<float>(0.0, r + config_.laser_stdev * laser_noise_(rng_));
        }
        rps.laserPublisher->publish(scanDataMsg);
    }
}

void Simulator::publishTransform() {
    geometry_msgs::msg::TransformStamped transform;
    tf2::Quaternion q;

    for (size_t i = 0; i < robot_pub_subs_.size(); ++i) {
        auto& rps = robot_pub_subs_[i];
        const auto pf = IndexToPrefix(i);

        // Publish simplified TF tree: map → odom → base_link → base_laser
        // map → odom (identity transform - map and odom frames are coincident in simulation)
        transform.header.stamp = node_->now();
        transform.header.frame_id = "/map";
        transform.child_frame_id = pf + "/odom";
        transform.transform.translation.x = 0.0;
        transform.transform.translation.y = 0.0;
        transform.transform.translation.z = 0.0;
        q.setRPY(0.0, 0.0, 0.0);
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();
        br->sendTransform(transform);

        // odom → base_link (robot pose in odom frame)
        transform.header.stamp = node_->now();
        transform.header.frame_id = pf + "/odom";
        transform.child_frame_id = pf + "/base_link";
        transform.transform.translation.x = rps.cur_loc.translation.x();
        transform.transform.translation.y = rps.cur_loc.translation.y();
        transform.transform.translation.z = 0.0;
        q.setRPY(0.0, 0.0, rps.cur_loc.angle);
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();
        br->sendTransform(transform);

        // base_link → base_laser (laser sensor position relative to robot)
        transform.header.stamp = node_->now();
        transform.header.frame_id = pf + "/base_link";
        transform.child_frame_id = pf + "/base_laser";
        transform.transform.translation.x = rps.laser_x;
        transform.transform.translation.y = rps.laser_y;
        transform.transform.translation.z = rps.laser_z;
        q.setRPY(0.0, 0.0, 0.0);
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();
        br->sendTransform(transform);
    }
}

void Simulator::update() {
    // Step the motion model forward one time step
    ++sim_step_count;
    sim_time += config_.dt;
    for (auto& rps : robot_pub_subs_) {
        rps.motion_model->Step(config_.dt);
        for (const Line2f& line : rps.motion_model->GetLines()) {
            map_.object_lines.push_back(line);
        }

        // Update the simulator with the motion model result.
        rps.cur_loc = rps.motion_model->GetPose();
        rps.vel = rps.motion_model->GetVel();

        // Publishing the ground truth pose
        localizationMsg.header.stamp = node_->now();
        localizationMsg.map = current_map_name_;
        localizationMsg.pose.x = rps.cur_loc.translation.x();
        localizationMsg.pose.y = rps.cur_loc.translation.y();
        localizationMsg.pose.theta = rps.cur_loc.angle;
        rps.localizationPublisher->publish(localizationMsg);
    }

    // Update all map objects and get their lines
    map_.object_lines.clear();
    for (size_t i = 0; i < objects.size(); i++) {
        objects[i]->Step(config_.dt);
        for (const Line2f& line : objects[i]->GetLines()) {
            map_.object_lines.push_back(line);
        }
    }
}

void Simulator::publishLocalization() {
    for (auto& rps : robot_pub_subs_) {
        localizationMsg.header.stamp = node_->now();
        localizationMsg.map = current_map_name_;
        localizationMsg.pose.x = rps.cur_loc.translation.x();
        localizationMsg.pose.y = rps.cur_loc.translation.y();
        localizationMsg.pose.theta = rps.cur_loc.angle;
        rps.localizationPublisher->publish(localizationMsg);
    }
}

void Simulator::CurrentMapCallback(const std_msgs::msg::String::SharedPtr msg) {
    if (current_map_name_ != msg->data) {
        RCLCPP_INFO(node_->get_logger(), "Simulator map changed to: %s", msg->data.c_str());
        current_map_name_ = msg->data;

        // Reload the map
        const std::string map_path = config_.maps_dir + "/" + msg->data + "/" + msg->data + ".vectormap.txt";
        if (!std::ifstream(map_path.c_str()).good()) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to locate map file at \"%s\"", map_path.c_str());
            return;
        }

        map_.Load(map_path);
        RCLCPP_INFO(node_->get_logger(), "Successfully loaded new map: %s", msg->data.c_str());
    }
}

void Simulator::Run() {
    update();
    publishOdometry();
    publishLaser();
    publishTransform();
    publishLocalization();
}
