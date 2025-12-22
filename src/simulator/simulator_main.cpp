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
 * \file    simulator_main.cpp
 * \brief   A simple simulator.
 * \author  Joydeep Biswas, (C) 2010
 */
//========================================================================

#include <stdio.h>
#include <iostream>

#include "glog/logging.h"
#include "gflags/gflags.h"
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include "ut_multirobot_sim/msg/simulator_state_msg.hpp"

#include "shared/util/timer.h"
#include "simulator.h"
#include "config_reader/config_reader.h"

using ut_multirobot_sim::msg::SimulatorStateMsg;

SimulatorStateMsg sim_state_;
bool sim_step_ = false;

SimulatorConfig LoadSimulatorConfig(const std::string& env_config,
                                    const std::string& maps_dir) {
    // Declare CONFIG_ macros BEFORE creating ConfigReader
    // ENVIRONMENT: Map and simulation settings
    CONFIG_STRING(map_name, "map_name");
    CONFIG_FLOAT(dt, "delta_t");
    CONFIG_STRING(current_map_topic, "current_map_topic");

    // SENSORS: Laser scan settings
    CONFIG_STRING(laser_topic, "laser_topic");
    CONFIG_STRING(laser_frame, "laser_frame");
    CONFIG_FLOAT(laser_stdev, "laser_noise_stddev");
    CONFIG_FLOAT(laser_angle_min, "laser_angle_min");
    CONFIG_FLOAT(laser_angle_max, "laser_angle_max");
    CONFIG_FLOAT(laser_angle_increment, "laser_angle_increment");
    CONFIG_FLOAT(laser_min_range, "laser_min_range");
    CONFIG_FLOAT(laser_max_range, "laser_max_range");

    // ROBOT FLEET
    CONFIG_STRINGLIST(robot_types, "robot_types");
    CONFIG_VECTOR3FLIST(start_poses, "start_poses");
    CONFIG_STRINGLIST(robot_configs, "robot_configs");

    // DYNAMIC OBJECTS
    CONFIG_STRINGLIST(short_term_object_configs, "short_term_object_config_list");
    CONFIG_STRINGLIST(human_configs, "human_config_list");

    // NOW create ConfigReader - it will populate the CONFIG_ variables
    config_reader::ConfigReader reader({env_config});

    SimulatorConfig config;
    config.maps_dir = maps_dir;
    config.map_name = CONFIG_map_name;
    config.dt = CONFIG_dt;
    config.current_map_topic = CONFIG_current_map_topic;

    config.laser_topic = CONFIG_laser_topic;
    config.laser_frame = CONFIG_laser_frame;
    config.laser_stdev = CONFIG_laser_stdev;
    config.laser_angle_min = CONFIG_laser_angle_min;
    config.laser_angle_max = CONFIG_laser_angle_max;
    config.laser_angle_increment = CONFIG_laser_angle_increment;
    config.laser_min_range = CONFIG_laser_min_range;
    config.laser_max_range = CONFIG_laser_max_range;

    // Build RobotConfig list
    if (CONFIG_robot_types.size() != CONFIG_start_poses.size() ||
        CONFIG_robot_types.size() != CONFIG_robot_configs.size()) {
        std::cerr << "ERROR: robot_types, start_poses, and robot_configs must have same length!" << std::endl;
        exit(1);
    }

    for (size_t i = 0; i < CONFIG_robot_types.size(); ++i) {
        RobotConfig robot;
        robot.type = CONFIG_robot_types[i];
        robot.start_pose = CONFIG_start_poses[i];
        robot.config_file = CONFIG_robot_configs[i];
        config.robots.push_back(robot);
    }

    config.short_term_object_configs = CONFIG_short_term_object_configs;
    config.human_configs = CONFIG_human_configs;

    return config;
}

DEFINE_string(config,
              "",
              "Path to simulator config (contains all settings) (required).");
DEFINE_string(maps_dir,
              "",
              "Path to maps directory.");

void SimStartStop(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data) {
        sim_state_.sim_state = SimulatorStateMsg::SIM_RUNNING;
    } else {
        sim_state_.sim_state = SimulatorStateMsg::SIM_STOPPED;
    }
}

void SimStep(const std_msgs::msg::Bool::SharedPtr msg) {
    // In case multiple step commands are received between sim updates, the
    // simulator should step at least once.
    sim_step_ = sim_step_ || msg->data;
}

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, false);

    // Check if config was provided
    if (FLAGS_config.empty()) {
        fprintf(stderr, "ERROR: --config flag is required. Please specify a simulator config file path.\n");
        fprintf(stderr, "Usage: %s --config=<path_to_config_file> [other options]\n", argv[0]);
        exit(1);
    }

    printf("\nUT Multi-Robot Simulator\n\n");

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("ut_multirobot_sim");

    auto sim_state_pub = node->create_publisher<SimulatorStateMsg>(
        "sim_state", 1);
    sim_state_.sim_state = SimulatorStateMsg::SIM_RUNNING;

    auto start_stop_sub = node->create_subscription<std_msgs::msg::Bool>(
        "sim_start_stop", 1, SimStartStop);

    auto step_sub = node->create_subscription<std_msgs::msg::Bool>(
        "sim_step", 1, SimStep);

    if (FLAGS_maps_dir.empty()) {
        FLAGS_maps_dir = ament_index_cpp::get_package_share_directory("amrl_maps");
    }
    CHECK_NE(FLAGS_maps_dir, string(""));

    // Load all configuration from single config file
    SimulatorConfig config = LoadSimulatorConfig(FLAGS_config, FLAGS_maps_dir);

    // Create simulator with loaded config
    Simulator simulator(config);
    if (!simulator.init(node)) {
        return 1;
    }

    // main loop
    RateLoop rate(1.0 / simulator.GetStepSize());
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        switch (sim_state_.sim_state) {
            case SimulatorStateMsg::SIM_RUNNING: {
                simulator.Run();
            } break;
            case SimulatorStateMsg::SIM_STOPPED: {
                // Do nothing unless stepping.
                if (sim_step_) {
                    simulator.Run();
                    // Disable stepping until a step message is received.
                    sim_step_ = false;
                }
            } break;
            default: {
                LOG(FATAL) << "Unexpected simulator state: " << sim_state_.sim_state;
            }
        }

        // Publish simulator state.
        sim_state_.sim_step_count = simulator.GetSimStepCount();
        sim_state_.sim_time = simulator.GetSimTime();
        sim_state_pub->publish(sim_state_);
        rate.Sleep();
    }

    printf("closing.\n");

    rclcpp::shutdown();
    return (0);
}
