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
  \file    robot_model.cpp
  \brief   C++ Interface: Abstract class for robot models
  \author  Jarrett Holtz, (C) 2020
  \email   jaholtz@cs.utexas.edu
*/
//========================================================================

#include "simulator/drive_models/robot_model.h"

namespace robot_model {

RobotModel::RobotModel() : EntityBase(),
                           vel_(0, {0, 0}),
                           last_cmd_(),
                           sim_time_at_last_cmd_(0),
                           current_sim_time_(0),
                           new_cmd_received_(false),
                           command_timeout(0.1),
                           cmd_timestamp_(0, 0, RCL_ROS_TIME),
                           node_(nullptr) {}

bool RobotModel::Init(rclcpp::Node::SharedPtr node, const std::string& topic_prefix, const std::string& drive_topic, double command_timeout) {
    node_ = node;
    this->command_timeout = command_timeout;
    last_cmd_.linear.x = 0.0;
    last_cmd_.linear.y = 0.0;
    last_cmd_.angular.z = 0.0;
    sim_time_at_last_cmd_ = 0.0;
    new_cmd_received_ = false;

    drive_subscriber_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        topic_prefix + drive_topic,
        20,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
            this->DriveCallback(msg);
        });

    return true;
}

void RobotModel::SetVel(const pose_2d::Pose2Df& vel) {
    vel_ = vel;
}

Pose2Df RobotModel::GetVel() {
    return vel_;
}

bool RobotModel::IsCommandTimedOut() {
    // Update sim_time_at_last_cmd_ when a new command was received
    if (new_cmd_received_) {
        sim_time_at_last_cmd_ = current_sim_time_;
        new_cmd_received_ = false;
    }
    return (current_sim_time_ > sim_time_at_last_cmd_ + this->command_timeout);
}

void RobotModel::StoreCommandTimestamp(const geometry_msgs::msg::Twist::SharedPtr msg) {
    (void)msg;
    cmd_timestamp_ = node_->now();
}

}  // namespace robot_model
