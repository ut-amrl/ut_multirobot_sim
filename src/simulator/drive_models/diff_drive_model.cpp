/*
 * The MIT License (MIT)
 * Copyright (c) 2011 William Woodall <wjwwood@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of oftware and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and ermission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <iostream>
#include <sstream>
#include <cmath>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "simulator/drive_models/diff_drive_model.h"
#include "shared/util/timer.h"

using Eigen::Rotation2Df;
using Eigen::Vector2f;
using Eigen::Vector3f;
using math_util::AngleMod;
using std::string;
using std::vector;

namespace diffdrive {

CONFIG_BOOL(invert_x, "invert_linear_vel_cmds");
CONFIG_BOOL(invert_z, "invert_angular_vel_cmds");
CONFIG_FLOAT(linear_pos_accel_limit, "linear_pos_accel_limit");
CONFIG_FLOAT(linear_neg_accel_limit, "linear_neg_accel_limit");
CONFIG_FLOAT(angular_pos_accel_limit, "angular_pos_accel_limit");
CONFIG_FLOAT(angular_neg_accel_limit, "angular_neg_accel_limit");
CONFIG_FLOAT(max_angular_vel, "max_angular");
CONFIG_FLOAT(max_linear_vel, "max_linear_vel");
CONFIG_FLOAT(linear_odom_scale, "linear_odom_scale");
CONFIG_FLOAT(angular_odom_scale, "angular_odom_scale");

// Drive topic standardized to "/cmd_vel" in simulator

DiffDriveModel::DiffDriveModel(const vector<string>& config_files) : RobotModel(),
                                                                     angular_error_(0, 1),
                                                                     config_reader_(config_files) {
    // ROS subscription initialized in Init()
    linear_vel_ = 0.0;
    angular_vel_ = 0.0;
    target_linear_vel_ = 0.0;
    target_angular_vel_ = 0.0;
    pose_.translation = Vector2f(0, 0);
    pose_.angle = 0;
}

void DiffDriveModel::Step(const double& dt) {
    // TODO(joydeepb): Make the 0.1 either a flag or config.
    static const double kMaxCommandAge = 0.1;
    if (IsCommandTimedOut(node_->now().seconds(), kMaxCommandAge)) {
        target_angular_vel_ = 0;
        target_linear_vel_ = 0;
    }
    rclcpp::Time current_time = node_->now();
    // Update the linear velocity based on the linear acceleration limits
    if (linear_vel_ < target_linear_vel_) {
        // Must increase linear speed
        if (CONFIG_linear_pos_accel_limit == 0.0 || target_linear_vel_ - linear_vel_ < CONFIG_linear_pos_accel_limit) {
            linear_vel_ = target_linear_vel_;
        } else {
            linear_vel_ += CONFIG_linear_pos_accel_limit;
        }
    } else if (linear_vel_ > target_linear_vel_) {
        // Must decrease linear speed
        if (CONFIG_linear_neg_accel_limit == 0.0 || linear_vel_ - target_linear_vel_ < CONFIG_linear_neg_accel_limit) {
            linear_vel_ = target_linear_vel_;
        } else {
            linear_vel_ -= CONFIG_linear_neg_accel_limit;
        }
    }

    // Update the angular velocity based on the angular acceleration limits
    if (angular_vel_ < target_angular_vel_) {
        // Must increase angular speed
        if (CONFIG_angular_pos_accel_limit == 0.0 || target_angular_vel_ - angular_vel_ < CONFIG_angular_pos_accel_limit) {
            angular_vel_ = target_angular_vel_;
        } else {
            angular_vel_ += CONFIG_angular_pos_accel_limit;
        }
    } else if (angular_vel_ > target_angular_vel_) {
        // Must decrease angular speed
        if (CONFIG_angular_neg_accel_limit == 0.0 || angular_vel_ - target_angular_vel_ < CONFIG_angular_neg_accel_limit) {
            angular_vel_ = target_angular_vel_;
        } else {
            angular_vel_ -= CONFIG_angular_neg_accel_limit;
        }
    }

    const float acc_times_dt = (linear_vel_ - vel_.translation.x());
    const float dx = vel_.translation.x() * dt + 0.5 * acc_times_dt * dt;

    vel_.translation.x() = linear_vel_;
    vel_.angle = angular_vel_;
    pose_.translation += geometry::Heading(pose_.angle) * dx;
    pose_.angle = AngleMod(pose_.angle + vel_.angle * dt);
    // Odometry publishing handled centrally by simulator
}

void DiffDriveModel::DriveCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->angular.z)) {
        printf("Ignoring non-finite drive values: linear.x=%f, angular.z=%f\n",
               msg->linear.x, msg->angular.z);
        return;
    }
    last_cmd_ = *msg;
    t_last_cmd_ = node_->now().seconds();

    double x = msg->linear.x, z = msg->angular.z;

    // invert motion, if needed
    if (CONFIG_invert_x) {
        x *= -1;
    }
    if (CONFIG_invert_z) {
        z *= -1;
    }

    // cut off velocities to their maximum
    if (CONFIG_max_linear_vel != 0.0) {
        if (fabs(x) > CONFIG_max_linear_vel) {
            x = (x > 0) ? CONFIG_max_linear_vel : -CONFIG_max_linear_vel;
        }
    }
    if (CONFIG_max_angular_vel != 0.0) {
        if (fabs(z) > CONFIG_max_angular_vel) {
            z = (z > 0) ? CONFIG_max_angular_vel : -CONFIG_max_angular_vel;
        }
    }
    target_linear_vel_ = x;
    target_angular_vel_ = z;
}

}  // namespace diffdrive
