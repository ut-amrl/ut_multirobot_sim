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
\file    simulator.h
\brief   C++ Interface: Simulator
\author  Joydeep Biswas, (C) 2011
*/
//========================================================================

#include <stdio.h>
#include <iostream>
#include <memory>
#include <random>
#include <vector>

#include "eigen3/Eigen/Dense"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "amrl_msgs/msg/ackermann_curvature_drive_msg.hpp"
#include "amrl_msgs/msg/localization2_d_msg.hpp"

#include "shared/math/geometry.h"
#include "shared/util/timer.h"
#include "simulator/vector_map.h"
#include "config_reader/config_reader.h"

#include "simulator/entities/entity_base.h"
#include "simulator/entities/human_object.h"
#include "simulator/drive_models/robot_model.h"
#include "simulator/entities/short_term_object.h"

#ifndef SIMULATOR_H
#define SIMULATOR_H

using namespace std;
using pose_2d::Pose2Df;

class Simulator {
    config_reader::ConfigReader reader_;
    config_reader::ConfigReader init_config_reader_;

    std::vector<std::unique_ptr<EntityBase>> objects;

    struct RobotPubSub {
        Pose2Df vel;
        Pose2Df cur_loc;

        rclcpp::Subscription<amrl_msgs::msg::Localization2DMsg>::SharedPtr initSubscriber;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometryTwistPublisher;
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserPublisher;
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr vizLaserPublisher;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr posMarkerPublisher;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr truePosePublisher;
        rclcpp::Publisher<amrl_msgs::msg::Localization2DMsg>::SharedPtr localizationPublisher;
        std::unique_ptr<robot_model::RobotModel> motion_model;

        visualization_msgs::msg::Marker robotPosMarker;
    };

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mapLinesPublisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr objectLinesPublisher;

    std::vector<RobotPubSub> robot_pub_subs_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> br;
    sensor_msgs::msg::LaserScan scanDataMsg;
    nav_msgs::msg::Odometry odometryTwistMsg;
    amrl_msgs::msg::Localization2DMsg localizationMsg;

    vector_map::VectorMap map_;

    visualization_msgs::msg::Marker lineListMarker;
    visualization_msgs::msg::Marker objectLinesMarker;

    static const float DT;
    geometry_msgs::msg::PoseStamped truePoseMsg;

    std::default_random_engine rng_;
    std::normal_distribution<float> laser_noise_;

    uint64_t sim_step_count;
    double sim_time;

    std::string robot_config_file_;
    std::string maps_dir_;

    rclcpp::Node::SharedPtr node_;

   private:
    void initVizMarker(visualization_msgs::msg::Marker& vizMarker, string ns, int id,
                       string type, geometry_msgs::msg::PoseStamped p,
                       geometry_msgs::msg::Point32 scale, double duration,
                       std::vector<float> color);
    void initSimulatorVizMarkers();
    void drawMap();
    void drawObjects();
    void InitalLocationCallback(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void DriveCallback(const amrl_msgs::msg::AckermannCurvatureDriveMsg::SharedPtr msg);
    void publishOdometry();
    void publishLaser();
    void publishVisualizationMarkers();
    void publishTransform();
    void publishLocalization();
    void update();
    void loadObject();

   public:
    Simulator() = delete;
    explicit Simulator(const std::string& env_config,
                       const std::string& robot_config,
                       const std::string& init_config,
                       const std::string& maps_dir);
    ~Simulator();
    bool init(rclcpp::Node::SharedPtr node);
    void Run();
    double GetSimTime() const { return sim_time; }
    uint64_t GetSimStepCount() const { return sim_step_count; }
    double GetStepSize() const;
};
#endif  // SIMULATOR_H
