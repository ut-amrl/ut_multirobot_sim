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

#include "amrl_msgs/msg/localization2_d_msg.hpp"

#include <string>
#include <vector>
#include <memory>

#include "eigen3/Eigen/Dense"
#include "shared/math/geometry.h"
#include "shared/util/timer.h"
#include "simulator/vector_map.h"

#include "simulator/entities/entity_base.h"
#include "simulator/entities/human_object.h"
#include "simulator/drive_models/robot_model.h"
#include "simulator/entities/short_term_object.h"

#ifndef SIMULATOR_H
#define SIMULATOR_H

using namespace std;
using pose_2d::Pose2Df;

// Drive models load their configurations directly from config files

// Individual robot configuration
struct RobotConfig {
    string type;                 // Robot type (DIFF_DRIVE, ACKERMANN_DRIVE, OMNIDIRECTIONAL_DRIVE)
    Eigen::Vector3f start_pose;  // Initial pose (x, y, theta)
    string config_file;          // Path to robot-specific config file
};

// Configuration structure passed from simulator_main to Simulator
struct SimulatorConfig {
    // Environment: map and simulation settings
    float dt;         // Simulation timestep
    string map_name;  // Map basename (e.g., "UT_Campus")
    string maps_dir;  // Directory containing map files

    // Sensors: laser scan settings
    string laser_topic;                      // Laser scan topic name
    string laser_frame;                      // Laser frame ID
    float laser_angle_min, laser_angle_max;  // Laser angular range
    float laser_angle_increment;             // Laser angular resolution
    float laser_min_range, laser_max_range;  // Laser distance range
    float laser_stdev;                       // Laser noise std deviation

    // Robot fleet: list of robots to simulate
    vector<RobotConfig> robots;  // Each robot with type, pose, and config file

    // Dynamic objects: humans and short-term obstacles
    vector<string> short_term_object_configs;  // Short-term object config files
    vector<string> human_configs;              // Human config files
};

class Simulator {
    // Loaded configuration
    SimulatorConfig config_;

    // Dynamic objects (humans, obstacles) in simulation
    std::vector<std::unique_ptr<EntityBase>> objects;

    // Per-robot publishers/subscribers and state
    struct RobotPubSub {
        Pose2Df vel;      // Current velocity (linear + angular)
        Pose2Df cur_loc;  // Current pose (x,y,theta)

        // Robot geometry (loaded from robot's config file)
        float car_length, car_width;      // Robot dimensions
        float laser_x, laser_y, laser_z;  // Laser position relative to base_link

        // ROS interfaces
        rclcpp::Subscription<amrl_msgs::msg::Localization2DMsg>::SharedPtr initSubscriber;      // Initial pose reset
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometryTwistPublisher;           // Odometry with covariances
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserPublisher;               // Simulated laser scans
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr posMarkerPublisher;       // Robot visualization
        rclcpp::Publisher<amrl_msgs::msg::Localization2DMsg>::SharedPtr localizationPublisher;  // Ground truth localization with map
        std::unique_ptr<robot_model::RobotModel> motion_model;                                  // Kinematics model

        visualization_msgs::msg::Marker robotPosMarker;  // RViz marker for robot
    };

    // Global visualization publishers
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mapLinesPublisher;     // Map visualization
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr objectLinesPublisher;  // Dynamic objects

    // All robot states and interfaces
    std::vector<RobotPubSub> robot_pub_subs_;

    // TF broadcasting for coordinate frames
    std::shared_ptr<tf2_ros::TransformBroadcaster> br;

    // Reusable message objects
    sensor_msgs::msg::LaserScan scanDataMsg;            // Laser scan data
    nav_msgs::msg::Odometry odometryTwistMsg;           // Odometry data
    amrl_msgs::msg::Localization2DMsg localizationMsg;  // Localization data

    // Map and environment
    vector_map::VectorMap map_;  // Vector map for collision detection

    // Visualization markers
    visualization_msgs::msg::Marker lineListMarker;     // Map lines
    visualization_msgs::msg::Marker objectLinesMarker;  // Object lines

    // Simulation timing
    static const float DT;  // Fixed timestep

    // Random number generation for sensor noise
    std::default_random_engine rng_;
    std::normal_distribution<float> laser_noise_;

    // Simulation state
    uint64_t sim_step_count;  // Current simulation step
    double sim_time;          // Current simulation time

    // ROS2 node handle
    rclcpp::Node::SharedPtr node_;

   private:
    // Visualization helpers
    void initVizMarker(visualization_msgs::msg::Marker& vizMarker, string ns, int id,  // Initialize RViz marker
                       string type, geometry_msgs::msg::PoseStamped p,
                       geometry_msgs::msg::Point32 scale, double duration,
                       std::vector<float> color);
    void initSimulatorVizMarkers();  // Setup visualization markers
    void drawMap();                  // Draw map lines in RViz
    void drawObjects();              // Draw dynamic objects in RViz

    // Publishing methods
    void publishOdometry();              // Publish odometry for all robots
    void publishLaser();                 // Publish laser scans for all robots
    void publishVisualizationMarkers();  // Publish RViz markers
    void publishTransform();             // Publish TF transforms
    void publishLocalization();          // Publish localization data

    // Simulation core
    void update();      // Step physics forward
    void loadObject();  // Load dynamic objects from config

   public:
    Simulator() = delete;
    explicit Simulator(const SimulatorConfig& config);           // Constructor with loaded config
    ~Simulator();                                                // Destructor
    bool init(rclcpp::Node::SharedPtr node);                     // Initialize ROS interfaces
    void Run();                                                  // Main simulation loop
    double GetSimTime() const { return sim_time; }               // Get current simulation time
    uint64_t GetSimStepCount() const { return sim_step_count; }  // Get simulation step count
    double GetStepSize() const { return config_.dt; }            // Get simulation timestep
};
#endif  // SIMULATOR_H
