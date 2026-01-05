#include "simulator/drive_models/omnidirectional_model.h"

#include <cmath>
#include <cstdio>

#include "config_reader/config_reader.h"
#include "eigen3/Eigen/Geometry"
#include "shared/math/math_util.h"

using Eigen::Rotation2Df;
using Eigen::Vector2f;
using math_util::AngleMod;
using math_util::Sign;

namespace omnidrive {

OmnidirectionalModel::OmnidirectionalModel(const std::string& config_file) : RobotModel() {
    CONFIG_FLOAT(max_accel, "max_accel");
    CONFIG_FLOAT(max_angle_accel, "max_angular_accel");
    CONFIG_FLOAT(max_speed, "max_speed");
    CONFIG_FLOAT(max_angle_vel, "max_angular_vel");
    CONFIG_BOOL(apply_limits, "apply_limits");

    // Load config from file
    config_reader::ConfigReader reader({config_file});

    max_accel_ = CONFIG_max_accel;
    max_angle_accel_ = CONFIG_max_angle_accel;
    max_speed_ = CONFIG_max_speed;
    max_angle_vel_ = CONFIG_max_angle_vel;
    apply_limits_ = CONFIG_apply_limits;
    // ROS subscription initialized in Init()
}

void OmnidirectionalModel::DriveCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->linear.y) || !std::isfinite(msg->angular.z)) {
        std::printf("Ignoring non-finite drive values: linear.x=%f, linear.y=%f, angular.z=%f\n",
                    msg->linear.x, msg->linear.y, msg->angular.z);
        return;
    }
    StoreCommandTimestamp(msg);
    last_cmd_ = *msg;
    new_cmd_received_ = true;
}

// TODO(jaholtz) Add noise
void OmnidirectionalModel::Step(const double& dt) {
    // Check command timeout (uses simulation time)
    if (IsCommandTimedOut()) {
        last_cmd_.linear.x = 0;
        last_cmd_.linear.y = 0;
        last_cmd_.angular.z = 0;
    }

    if (!apply_limits_) {
        vel_.translation.x() = last_cmd_.linear.x;
        vel_.translation.y() = last_cmd_.linear.y;
        vel_.angle = last_cmd_.angular.z;
        pose_.translation += Rotation2Df(pose_.angle) * vel_.translation * dt;
        pose_.angle = AngleMod(pose_.angle + vel_.angle * dt);
        return;
    }

    // Cap Velocity to max speed
    Vector2f desired_vel(last_cmd_.linear.x, last_cmd_.linear.y);
    if (desired_vel.norm() > max_speed_) {
        desired_vel = max_speed_ * desired_vel.normalized();
    }

    // Cap acceleration to max accel
    Vector2f delta_v = desired_vel - vel_.translation;
    const float max_accel = max_accel_ * dt;
    if (delta_v.norm() > max_accel) {
        delta_v = max_accel * delta_v.normalized();
    }

    // Update translational velocity
    vel_.translation += delta_v;

    // Cap the rotational velocity and acceleration
    float desired_ang_vel = last_cmd_.angular.z;
    if (std::fabs(desired_ang_vel) > max_angle_vel_) {
        desired_ang_vel = Sign(desired_ang_vel) * max_angle_vel_;
    }
    const float max_angle_accel = max_angle_accel_ * dt;
    float delta_ang_v = desired_ang_vel - vel_.angle;
    if (std::fabs(delta_ang_v) > max_angle_accel) {  // FIX: Check delta, not desired!
        delta_ang_v = Sign(delta_ang_v) * max_angle_accel;
    }
    vel_.angle += delta_ang_v;

    pose_.translation += Rotation2Df(pose_.angle) * vel_.translation * dt;
    pose_.angle = AngleMod(pose_.angle + vel_.angle * dt);
    // Odometry publishing handled centrally by simulator
}

}  // namespace omnidrive
