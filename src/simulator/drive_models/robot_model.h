//========================================================================
//      This software is free: you can redistribute it and/or modif    y
//  it under the terms of the GNU Lesser General Public License Vers    ion 3,
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
  \file    robot_model.h
  \brief   C++ Interface: Abstract class for robots built on entity_base
  \author  Jarrett Holtz, (C) 2020
  \email   jaholtz@cs.utexas.edu
*/
//========================================================================

#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

#include "simulator/entities/entity_base.h"

#ifndef SRC_SIMULATOR_ROBOT_MODEL_H_
#define SRC_SIMULATOR_ROBOT_MODEL_H_

namespace robot_model {
class RobotModel : public EntityBase {
   protected:
    Pose2Df vel_;
    geometry_msgs::msg::Twist last_cmd_;
    double sim_time_at_last_cmd_;  // Simulation time when last command was received
    double current_sim_time_;      // Current simulation time (set by simulator)
    bool new_cmd_received_;        // Flag to track new command arrival
    double command_timeout;        // Command timeout duration [seconds]
    rclcpp::Time cmd_timestamp_;   // Timestamp from command message header
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr drive_subscriber_;
    rclcpp::Node::SharedPtr node_;

    // Standardized drive callback for all models
    virtual void DriveCallback(const geometry_msgs::msg::Twist::SharedPtr msg) = 0;

    // Store command timestamp for external calibration
    void StoreCommandTimestamp(const geometry_msgs::msg::Twist::SharedPtr msg);

    // Check if drive command has timed out (using simulation time)
    bool IsCommandTimedOut();

   public:
    RobotModel();
    virtual ~RobotModel() = default;

    // Initialize with ROS node and topic
    virtual bool Init(rclcpp::Node::SharedPtr node, const std::string& topic_prefix, const std::string& drive_topic, double command_timeout = 0.1);

    virtual void SetVel(const pose_2d::Pose2Df& vel);
    virtual pose_2d::Pose2Df GetVel();

    // Set current simulation time (called by simulator before Step)
    void SetCurrentSimTime(double sim_time) { current_sim_time_ = sim_time; }

    // Get the timestamp of the last command for external calibration
    rclcpp::Time GetLastCommandTimestamp() const { return cmd_timestamp_; }
};
}  // namespace robot_model

#endif  // SRC_SIMULATOR_ROBOT_MODEL_H_
