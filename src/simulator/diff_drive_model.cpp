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

#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "diff_drive_model.h"
#include "shared/util/timer.h"

using Eigen::Vector2f;
using Eigen::Vector3f;
using Eigen::Rotation2Df;
using std::vector;
using std::string;
using math_util::AngleMod;

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

CONFIG_STRING(drive_topic, "drive_callback_topic");
CONFIG_STRING(odom_topic, "diff_drive_odom_topic");

DiffDriveModel::DiffDriveModel(const vector<string>& config_files, ros::NodeHandle* n) :
    RobotModel(),
    last_cmd_(),
    t_last_cmd_(0),
    angular_error_(0, 1),
    config_reader_(config_files) {

    drive_subscriber_ = n->subscribe(
      CONFIG_drive_topic,
      1,
      &DiffDriveModel::DriveCallback,
      this);
    odom_publisher_ = n->advertise<nav_msgs::Odometry>(CONFIG_odom_topic, 1);
    linear_vel_ = 0.0;
    angular_vel_ = 0.0;
    target_linear_vel_ = 0.0;
    target_angular_vel_ = 0.0;
    pose_.translation = Vector2f(0,0);
    pose_.angle = 0;
    odom_msg_.header.frame_id = "odom";
    odom_msg_.child_frame_id = "base_footprint";

}

void DiffDriveModel::Step(const double &dt) {
  // TODO(joydeepb): Make the 0.1 either a flag or config.
  if (t_last_cmd_ < GetMonotonicTime() - 0.1) {
    target_angular_vel_ = 0;
    target_linear_vel_ = 0;
  }
    ros::Time current_time = ros::Time::now();
    // Update the linear velocity based on the linear acceleration limits
    if (linear_vel_ < target_linear_vel_) {
        // Must increase linear speed
        if (CONFIG_linear_pos_accel_limit == 0.0
            || target_linear_vel_ - linear_vel_ < CONFIG_linear_pos_accel_limit) {
            linear_vel_ = target_linear_vel_;
        }
        else {
             linear_vel_ += CONFIG_linear_pos_accel_limit;
        }
    } else if (linear_vel_ > target_linear_vel_) {
        // Must decrease linear speed
        if (CONFIG_linear_neg_accel_limit == 0.0
            || linear_vel_ - target_linear_vel_ < CONFIG_linear_neg_accel_limit) {
            linear_vel_ = target_linear_vel_;
        }
        else {
             linear_vel_ -= CONFIG_linear_neg_accel_limit;
        }
    }

    // Update the angular velocity based on the angular acceleration limits
    if (angular_vel_ < target_angular_vel_) {
        // Must increase angular speed
        if (CONFIG_angular_pos_accel_limit == 0.0
            || target_angular_vel_ - angular_vel_ < CONFIG_angular_pos_accel_limit) {
            angular_vel_ = target_angular_vel_;
        }
        else {
             angular_vel_ += CONFIG_angular_pos_accel_limit;
        }
    } else if (angular_vel_ > target_angular_vel_) {
        // Must decrease angular speed
        if (CONFIG_angular_neg_accel_limit == 0.0
            || angular_vel_ - target_angular_vel_ < CONFIG_angular_neg_accel_limit) {
            angular_vel_ = target_angular_vel_;
        }
        else {
             angular_vel_ -= CONFIG_angular_neg_accel_limit;
        }
    }

    vel_.translation.x() = linear_vel_;
    vel_.angle = angular_vel_;
    pose_.translation += Rotation2Df(pose_.angle) * vel_.translation * dt;
    pose_.angle = AngleMod(pose_.angle + vel_.angle * dt);

    // Create a Quaternion from the angle
    quat_ = tf::createQuaternionMsgFromYaw(pose_.angle);
    last_time_ = current_time;

    PublishOdom(dt);
}

void DiffDriveModel::PublishOdom(const float dt) {
    odom_msg_.header.stamp = last_time_;
    odom_msg_.pose.pose.position.x = pose_.translation.x();
    odom_msg_.pose.pose.position.y = pose_.translation.y();
    odom_msg_.pose.pose.position.z = 0.0;
    odom_msg_.pose.pose.orientation = quat_;
    odom_msg_.pose.covariance[0] = 0.00001;
    odom_msg_.pose.covariance[7] = 0.00001;
    odom_msg_.pose.covariance[14] = 1000000000000.0;
    odom_msg_.pose.covariance[21] = 1000000000000.0;
    odom_msg_.pose.covariance[28] = 1000000000000.0;
    odom_msg_.pose.covariance[35] = 0.001;

    odom_msg_.twist.twist.linear.x = vel_.translation.x();
    odom_msg_.twist.twist.linear.y = vel_.translation.y();
    odom_msg_.twist.twist.angular.z = vel_.angle;

    odom_publisher_.publish(odom_msg_);
}

void DiffDriveModel::DriveCallback(const geometry_msgs::Twist& msg) {
    last_cmd_ = msg;
    t_last_cmd_ = GetMonotonicTime();
    double x = msg.linear.x, z = msg.angular.z;

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
};
