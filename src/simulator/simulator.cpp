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

#include <libgen.h>
#include <math.h>
#include <memory>
#include <stdio.h>
#include <random>

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
#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/ros/ros_helpers.h"
#include "shared/util/timer.h"
#include "amrl_msgs/msg/localization2_d_msg.hpp"
#include "vector_map.h"

DEFINE_bool(localize, false, "Publish localization");

using ackermann::AckermannModel;
using diffdrive::DiffDriveModel;
using Eigen::Rotation2Df;
using Eigen::Vector2f;
using geometry::Heading;
using geometry::Line2f;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using human::HumanObject;
using math_util::AngleMod;
using math_util::DegToRad;
using math_util::RadToDeg;
using omnidrive::OmnidirectionalModel;
using std::atan2;
using vector_map::VectorMap;

// Used for visualizations
CONFIG_FLOAT(car_length, "car_length");
CONFIG_FLOAT(car_width, "car_width");
CONFIG_FLOAT(car_height, "car_height");
CONFIG_FLOAT(rear_axle_offset, "rear_axle_offset");
// Used for transforms
CONFIG_FLOAT(laser_x, "laser_loc.x");
CONFIG_FLOAT(laser_y, "laser_loc.y");
CONFIG_FLOAT(laser_z, "laser_loc.z");
// Timestep size
CONFIG_FLOAT(DT, "delta_t");
CONFIG_FLOAT(laser_stdev, "laser_noise_stddev");
// TF publications
CONFIG_BOOL(publish_tfs, "publish_tfs");
CONFIG_BOOL(publish_map_to_odom, "publish_map_to_odom");
CONFIG_BOOL(publish_foot_to_base, "publish_foot_to_base");

// Used for topic names and robot specs
CONFIG_STRINGLIST(robot_types, "robot_types");
CONFIG_STRING(laser_topic, "laser_topic");
CONFIG_STRING(laser_frame, "laser_frame");

// Laser scanner parameters.
CONFIG_FLOAT(laser_angle_min, "laser_angle_min");
CONFIG_FLOAT(laser_angle_max, "laser_angle_max");
CONFIG_FLOAT(laser_angle_increment, "laser_angle_increment");
CONFIG_FLOAT(laser_min_range, "laser_min_range");
CONFIG_FLOAT(laser_max_range, "laser_max_range");

CONFIG_STRING(map_name, "map_name");
// Initial location
CONFIG_VECTOR3FLIST(start_poses, "start_poses");
CONFIG_STRINGLIST(short_term_object_config_list, "short_term_object_config_list");
CONFIG_STRINGLIST(human_config_list, "human_config_list");

Simulator::Simulator(const std::string& env_config,
                     const std::string& robot_config,
                     const std::string& init_config,
                     const std::string& maps_dir) : reader_({env_config, robot_config}),
                                                    init_config_reader_({init_config}),
                                                    laser_noise_(0, 1),
                                                    sim_step_count(0),
                                                    sim_time(0.0),
                                                    robot_config_file_(robot_config),
                                                    maps_dir_(maps_dir) {
    truePoseMsg.header.frame_id = "map";
    if (CONFIG_map_name == "") {
        std::cerr << "Failed to load map from init config file '"
                  << init_config << "'" << std::endl;
        exit(1);
    }
}

Simulator::~Simulator() {}

double Simulator::GetStepSize() const {
    return CONFIG_DT;
}

robot_model::RobotModel* MakeMotionModel(const std::string& robot_type,
                                         rclcpp::Node::SharedPtr node,
                                         const std::string& topic_prefix,
                                         const std::string& robot_config) {
    if (robot_type == "ACKERMANN_DRIVE") {
        return new AckermannModel({robot_config}, node);
    } else if (robot_type == "OMNIDIRECTIONAL_DRIVE") {
        return new OmnidirectionalModel({robot_config}, node);
    } else if (robot_type == "DIFF_DRIVE") {
        return new DiffDriveModel({robot_config}, node, topic_prefix);
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

    scanDataMsg.header.frame_id = CONFIG_laser_frame;
    scanDataMsg.angle_min = CONFIG_laser_angle_min;
    scanDataMsg.angle_max = CONFIG_laser_angle_max;
    scanDataMsg.angle_increment = CONFIG_laser_angle_increment;
    scanDataMsg.range_min = CONFIG_laser_min_range;
    scanDataMsg.range_max = CONFIG_laser_max_range;
    scanDataMsg.intensities.clear();
    scanDataMsg.time_increment = 0.0;
    scanDataMsg.scan_time = 0.05;

    odometryTwistMsg.header.frame_id = "odom";
    odometryTwistMsg.child_frame_id = "base_footprint";

    if (CONFIG_robot_types.size() != CONFIG_start_poses.size()) {
        std::cerr << "Robot type and robot start pose lists are"
                     "not the same size!"
                  << std::endl;
        return false;
    }

    initSimulatorVizMarkers();
    map_.Load(maps_dir_ + "/" + CONFIG_map_name);
    drawMap();

    // Create motion model based on robot type
    for (size_t i = 0; i < CONFIG_start_poses.size(); ++i) {
        const auto& robot_type = CONFIG_robot_types.at(i);
        const auto& start_pose = CONFIG_start_poses.at(i);
        const auto pf = IndexToPrefix(i);
        auto* mm = MakeMotionModel(robot_type, node_, pf, robot_config_file_);
        if (mm == nullptr) {
            return false;
        }
        mm->SetPose(Pose2Df(start_pose.z(), {start_pose.x(), start_pose.y()}));

        robot_pub_subs_.emplace_back(RobotPubSub());
        auto& rps = robot_pub_subs_.back();
        rps.motion_model = std::unique_ptr<robot_model::RobotModel>(mm);

        rps.initSubscriber = node_->create_subscription<amrl_msgs::msg::Localization2DMsg>(
            pf + "/initialpose", 1,
            [&rps](const amrl_msgs::msg::Localization2DMsg::SharedPtr msg) {
                const Vector2f loc(msg->pose.x, msg->pose.y);
                const float angle = msg->pose.theta;
                rps.motion_model->SetPose({angle, loc});
            });
        rps.odometryTwistPublisher = node_->create_publisher<nav_msgs::msg::Odometry>(pf + "/odom", 1);
        rps.laserPublisher = node_->create_publisher<sensor_msgs::msg::LaserScan>(pf + CONFIG_laser_topic, 1);
        rps.vizLaserPublisher = node_->create_publisher<sensor_msgs::msg::LaserScan>(pf + "/scan", 1);
        rps.posMarkerPublisher = node_->create_publisher<visualization_msgs::msg::Marker>(
            pf + "/simulator_visualization", 6);
        rps.truePosePublisher = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
            pf + "/simulator_true_pose", 1);

        if (FLAGS_localize) {
            rps.localizationPublisher = node_->create_publisher<amrl_msgs::msg::Localization2DMsg>(
                pf + "/localization", 1);
            localizationMsg.header.frame_id = "map";
        }
    }

    mapLinesPublisher = node_->create_publisher<visualization_msgs::msg::Marker>("/simulator_visualization", 6);
    objectLinesPublisher = node_->create_publisher<visualization_msgs::msg::Marker>("/simulator_visualization", 6);

    br = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

    this->loadObject();
    return true;
}

// TODO(yifeng): Change this into a general way
void Simulator::loadObject() {
    for (const string& config_str : CONFIG_short_term_object_config_list) {
        objects.push_back(
            std::unique_ptr<ShortTermObject>(new ShortTermObject(config_str)));
    }

    for (const string& config_str : CONFIG_human_config_list) {
        objects.push_back(
            std::unique_ptr<HumanObject>(new HumanObject({config_str})));
    }
}

/**
 * Helper method that initializes visualization_msgs::Marker parameters
 * @param vizMarker   pointer to the visualization_msgs::Marker object
 * @param ns          namespace for marker (string)
 * @param id          id of marker (int) - must be unique for each marker;
 *                      0, 1, and 2 are already used
 * @param type        specifies type of marker (string); available options:
 *                      arrow (default), cube, sphere, cylinder, linelist,
 *                      linestrip, points
 * @param p           stamped pose to define location and frame of marker
 * @param scale       scale of the marker; see visualization_msgs::Marker
 *                      documentation for details on the parameters
 * @param duration    lifetime of marker in RViz (double); use duration of 0.0
 *                      for infinite lifetime
 * @param color       vector of 4 float values representing color of marker;
 *                    0: red, 1: green, 2: blue, 3: alpha
 */
void Simulator::initVizMarker(visualization_msgs::msg::Marker& vizMarker, string ns,
                              int id, string type, geometry_msgs::msg::PoseStamped p,
                              geometry_msgs::msg::Point32 scale, double duration, vector<float> color) {
    vizMarker.header.frame_id = p.header.frame_id;
    vizMarker.header.stamp = node_->now();

    vizMarker.ns = ns;
    vizMarker.id = id;

    if (type == "arrow") {
        vizMarker.type = visualization_msgs::msg::Marker::ARROW;
    } else if (type == "cube") {
        vizMarker.type = visualization_msgs::msg::Marker::CUBE;
    } else if (type == "sphere") {
        vizMarker.type = visualization_msgs::msg::Marker::SPHERE;
    } else if (type == "cylinder") {
        vizMarker.type = visualization_msgs::msg::Marker::CYLINDER;
    } else if (type == "linelist") {
        vizMarker.type = visualization_msgs::msg::Marker::LINE_LIST;
    } else if (type == "linestrip") {
        vizMarker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    } else if (type == "points") {
        vizMarker.type = visualization_msgs::msg::Marker::POINTS;
    } else {
        vizMarker.type = visualization_msgs::msg::Marker::ARROW;
    }

    vizMarker.pose = p.pose;
    vizMarker.points.clear();
    vizMarker.scale.x = scale.x;
    vizMarker.scale.y = scale.y;
    vizMarker.scale.z = scale.z;

    vizMarker.lifetime = rclcpp::Duration::from_seconds(duration);

    vizMarker.color.r = color.at(0);
    vizMarker.color.g = color.at(1);
    vizMarker.color.b = color.at(2);
    vizMarker.color.a = color.at(3);

    vizMarker.action = visualization_msgs::msg::Marker::ADD;
}

void Simulator::initSimulatorVizMarkers() {
    geometry_msgs::msg::PoseStamped p;
    geometry_msgs::msg::Point32 scale;
    vector<float> color;
    color.resize(4);

    p.header.frame_id = "/map";

    p.pose.orientation.w = 1.0;
    scale.x = 0.02;
    scale.y = 0.0;
    scale.z = 0.0;
    color[0] = 66.0 / 255.0;
    color[1] = 134.0 / 255.0;
    color[2] = 244.0 / 255.0;
    color[3] = 1.0;
    initVizMarker(lineListMarker, "map_lines", 0, "linelist", p, scale, 0.0,
                  color);

    for (auto& rps : robot_pub_subs_) {
        p.pose.position.z = 0.5 * CONFIG_car_height;
        scale.x = CONFIG_car_length;
        scale.y = CONFIG_car_width;
        scale.z = CONFIG_car_height;
        color[0] = 94.0 / 255.0;
        color[1] = 156.0 / 255.0;
        color[2] = 255.0 / 255.0;
        color[3] = 0.8;
        initVizMarker(rps.robotPosMarker, "robot_position", 1, "cube", p, scale, 0.0,
                      color);
    }

    p.pose.orientation.w = 1.0;
    scale.x = 0.02;
    scale.y = 0.0;
    scale.z = 0.0;
    color[0] = 244.0 / 255.0;
    color[1] = 0.0 / 255.0;
    color[2] = 156.0 / 255.0;
    color[3] = 1.0;
    initVizMarker(objectLinesMarker, "object_lines", 0, "linelist", p, scale,
                  0.0, color);
}

void Simulator::drawMap() {
    ros_helpers::ClearMarker(&lineListMarker);
    for (const Line2f& l : map_.lines) {
        ros_helpers::DrawEigen2DLine(l.p0, l.p1, &lineListMarker);
    }
}

void Simulator::drawObjects() {
    // draw objects
    ros_helpers::ClearMarker(&objectLinesMarker);
    for (const Line2f& l : map_.object_lines) {
        ros_helpers::DrawEigen2DLine(l.p0, l.p1, &objectLinesMarker);
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
        odometryTwistMsg.twist.twist.angular.x = 0.0;
        odometryTwistMsg.twist.twist.angular.y = 0.0;
        odometryTwistMsg.twist.twist.angular.z = rps.vel.angle;
        odometryTwistMsg.twist.twist.linear.x = rps.vel.translation.x();
        odometryTwistMsg.twist.twist.linear.y = rps.vel.translation.y();
        odometryTwistMsg.twist.twist.linear.z = 0.0;

        rps.odometryTwistPublisher->publish(odometryTwistMsg);

        // TODO(jaholtz) visualization should not always be based on car
        // parameters
        rps.robotPosMarker.pose.position.x =
            rps.cur_loc.translation.x() - cos(rps.cur_loc.angle) * CONFIG_rear_axle_offset;
        rps.robotPosMarker.pose.position.y =
            rps.cur_loc.translation.y() - sin(rps.cur_loc.angle) * CONFIG_rear_axle_offset;
        rps.robotPosMarker.pose.position.z = 0.5 * CONFIG_car_height;
        rps.robotPosMarker.pose.orientation.x = robotQ.x();
        rps.robotPosMarker.pose.orientation.y = robotQ.y();
        rps.robotPosMarker.pose.orientation.z = robotQ.z();
        rps.robotPosMarker.pose.orientation.w = robotQ.w();
    }
}

void Simulator::publishLaser() {
    for (size_t i = 0; i < robot_pub_subs_.size(); ++i) {
        auto& rps = robot_pub_subs_[i];
        scanDataMsg.header.stamp = node_->now();
        scanDataMsg.header.frame_id = IndexToPrefix(i) + CONFIG_laser_frame;
        const Vector2f laserRobotLoc(CONFIG_laser_x, CONFIG_laser_y);
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
            r = max<float>(0.0, r + CONFIG_laser_stdev * laser_noise_(rng_));
        }

        // TODO Avoid publishing laser twice.
        // Currently publishes once for the visualizer and once for robot
        // requirements.
        rps.laserPublisher->publish(scanDataMsg);
        rps.vizLaserPublisher->publish(scanDataMsg);
    }
}

void Simulator::publishTransform() {
    if (!CONFIG_publish_tfs) {
        return;
    }

    geometry_msgs::msg::TransformStamped transform;
    tf2::Quaternion q;

    for (size_t i = 0; i < robot_pub_subs_.size(); ++i) {
        auto& rps = robot_pub_subs_[i];
        const auto pf = IndexToPrefix(i);

        if (CONFIG_publish_map_to_odom) {
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
        }

        transform.header.stamp = node_->now();
        transform.header.frame_id = pf + "/odom";
        transform.child_frame_id = pf + "/base_footprint";
        transform.transform.translation.x = rps.cur_loc.translation.x();
        transform.transform.translation.y = rps.cur_loc.translation.y();
        transform.transform.translation.z = 0.0;
        q.setRPY(0.0, 0.0, rps.cur_loc.angle);
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();
        br->sendTransform(transform);

        if (CONFIG_publish_foot_to_base) {
            transform.header.stamp = node_->now();
            transform.header.frame_id = pf + "/base_footprint";
            transform.child_frame_id = pf + "/base_link";
            transform.transform.translation.x = 0.0;
            transform.transform.translation.y = 0.0;
            transform.transform.translation.z = 0.0;
            q.setRPY(0.0, 0.0, 0.0);
            transform.transform.rotation.x = q.x();
            transform.transform.rotation.y = q.y();
            transform.transform.rotation.z = q.z();
            transform.transform.rotation.w = q.w();
            br->sendTransform(transform);
        }

        transform.header.stamp = node_->now();
        transform.header.frame_id = pf + "/base_link";
        transform.child_frame_id = pf + "/base_laser";
        transform.transform.translation.x = CONFIG_laser_x;
        transform.transform.translation.y = CONFIG_laser_y;
        transform.transform.translation.z = CONFIG_laser_z;
        q.setRPY(0.0, 0.0, 0.0);
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();
        br->sendTransform(transform);
    }
}

void Simulator::publishVisualizationMarkers() {
    mapLinesPublisher->publish(lineListMarker);
    objectLinesPublisher->publish(objectLinesMarker);
    for (auto& rps : robot_pub_subs_) {
        rps.posMarkerPublisher->publish(rps.robotPosMarker);
    }
}

void Simulator::update() {
    // Step the motion model forward one time step
    ++sim_step_count;
    sim_time += CONFIG_DT;
    for (auto& rps : robot_pub_subs_) {
        rps.motion_model->Step(CONFIG_DT);
        for (const Line2f& line : rps.motion_model->GetLines()) {
            map_.object_lines.push_back(line);
        }

        // Update the simulator with the motion model result.
        rps.cur_loc = rps.motion_model->GetPose();
        rps.vel = rps.motion_model->GetVel();

        // Publishing the ground truth pose
        truePoseMsg.header.stamp = node_->now();
        truePoseMsg.pose.position.x = rps.cur_loc.translation.x();
        truePoseMsg.pose.position.y = rps.cur_loc.translation.y();
        truePoseMsg.pose.position.z = 0;
        truePoseMsg.pose.orientation.w = cos(0.5 * rps.cur_loc.angle);
        truePoseMsg.pose.orientation.z = sin(0.5 * rps.cur_loc.angle);
        truePoseMsg.pose.orientation.x = 0;
        truePoseMsg.pose.orientation.y = 0;
        rps.truePosePublisher->publish(truePoseMsg);
    }

    // Update all map objects and get their lines
    map_.object_lines.clear();
    for (size_t i = 0; i < objects.size(); i++) {
        objects[i]->Step(CONFIG_DT);
        for (const Line2f& line : objects[i]->GetLines()) {
            map_.object_lines.push_back(line);
        }
    }
    this->drawObjects();
}

string GetMapNameFromFilename(string path) {
    char path_cstring[path.length()];
    strcpy(path_cstring, path.c_str());
    const string file_name(basename(path_cstring));
    static const string suffix = ".vectormap.txt";
    size_t found = file_name.find(suffix);
    return file_name.substr(0, found);
}

void Simulator::publishLocalization() {
    for (auto& rps : robot_pub_subs_) {
        localizationMsg.header.stamp = node_->now();
        localizationMsg.map = GetMapNameFromFilename(map_.file_name);
        localizationMsg.pose.x = rps.cur_loc.translation.x();
        localizationMsg.pose.y = rps.cur_loc.translation.y();
        localizationMsg.pose.theta = rps.cur_loc.angle;
        rps.localizationPublisher->publish(localizationMsg);
    }
}

void Simulator::Run() {
    update();
    publishOdometry();
    publishLaser();
    publishVisualizationMarkers();
    publishTransform();

    if (FLAGS_localize) {
        publishLocalization();
    }
}
