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
#include "config_reader/config_reader.h"
#include "simulator/drive_models/diff_drive_model.h"
#include "shared/util/timer.h"

using Eigen::Rotation2Df;
using Eigen::Vector2f;
using Eigen::Vector3f;
using math_util::AngleMod;
using std::string;
using std::vector;

namespace diffdrive {

DiffDriveModel::DiffDriveModel(const std::string& config_file) : RobotModel() {
    CONFIG_BOOL(invert_linear_vel_cmds, "invert_linear_vel_cmds");
    CONFIG_BOOL(invert_angular_vel_cmds, "invert_angular_vel_cmds");
    CONFIG_FLOAT(linear_pos_accel_limit, "linear_pos_accel_limit");
    CONFIG_FLOAT(linear_neg_accel_limit, "linear_neg_accel_limit");
    CONFIG_FLOAT(angular_pos_accel_limit, "angular_pos_accel_limit");
    CONFIG_FLOAT(angular_neg_accel_limit, "angular_neg_accel_limit");
    CONFIG_FLOAT(max_angular_vel, "max_angular");
    CONFIG_FLOAT(max_linear_vel, "max_linear_vel");

    // Load config from file
    config_reader::ConfigReader reader({config_file});

    invert_linear_vel_cmds_ = CONFIG_invert_linear_vel_cmds;
    invert_angular_vel_cmds_ = CONFIG_invert_angular_vel_cmds;
    linear_pos_accel_limit_ = CONFIG_linear_pos_accel_limit;
    linear_neg_accel_limit_ = CONFIG_linear_neg_accel_limit;
    angular_pos_accel_limit_ = CONFIG_angular_pos_accel_limit;
    angular_neg_accel_limit_ = CONFIG_angular_neg_accel_limit;
    max_angular_vel_ = CONFIG_max_angular_vel;
    max_linear_vel_ = CONFIG_max_linear_vel;
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
    // Apply acceleration limits (multiply by dt to get velocity change per timestep)
    const float max_linear_dv_pos = linear_pos_accel_limit_ * dt;
    const float max_linear_dv_neg = linear_neg_accel_limit_ * dt;
    const float max_angular_dv_pos = angular_pos_accel_limit_ * dt;
    const float max_angular_dv_neg = angular_neg_accel_limit_ * dt;

    // Update linear velocity based on acceleration limits
    float linear_dv = target_linear_vel_ - linear_vel_;
    if (linear_dv > 0) {
        // Accelerating forward
        if (max_linear_dv_pos == 0.0 || linear_dv < max_linear_dv_pos) {
            linear_vel_ = target_linear_vel_;
        } else {
            linear_vel_ += max_linear_dv_pos;
        }
    } else if (linear_dv < 0) {
        // Decelerating / reversing
        if (max_linear_dv_neg == 0.0 || -linear_dv < max_linear_dv_neg) {
            linear_vel_ = target_linear_vel_;
        } else {
            linear_vel_ -= max_linear_dv_neg;
        }
    }

    // Update angular velocity based on acceleration limits
    float angular_dv = target_angular_vel_ - angular_vel_;
    if (angular_dv > 0) {
        // Turning CCW
        if (max_angular_dv_pos == 0.0 || angular_dv < max_angular_dv_pos) {
            angular_vel_ = target_angular_vel_;
        } else {
            angular_vel_ += max_angular_dv_pos;
        }
    } else if (angular_dv < 0) {
        // Turning CW
        if (max_angular_dv_neg == 0.0 || -angular_dv < max_angular_dv_neg) {
            angular_vel_ = target_angular_vel_;
        } else {
            angular_vel_ -= max_angular_dv_neg;
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
    if (invert_linear_vel_cmds_) {
        x *= -1;
    }
    if (invert_angular_vel_cmds_) {
        z *= -1;
    }

    // cut off velocities to their maximum
    if (max_linear_vel_ != 0.0) {
        if (fabs(x) > max_linear_vel_) {
            x = (x > 0) ? max_linear_vel_ : -max_linear_vel_;
        }
    }
    if (max_angular_vel_ != 0.0) {
        if (fabs(z) > max_angular_vel_) {
            z = (z > 0) ? max_angular_vel_ : -max_angular_vel_;
        }
    }
    target_linear_vel_ = x;
    target_angular_vel_ = z;
}

}  // namespace diffdrive
