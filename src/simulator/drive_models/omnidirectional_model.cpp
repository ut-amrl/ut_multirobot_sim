#include "simulator/drive_models/omnidirectional_model.h"
#include <eigen3/Eigen/src/Geometry/Rotation2D.h>
#include "shared/util/timer.h"
#include "shared/math/math_util.h"
#include <geometry_msgs/msg/twist.hpp>

using Eigen::Rotation2Df;
using Eigen::Vector2f;
using geometry::Heading;
using geometry_msgs::msg::Twist;
using math_util::AngleDiff;
using math_util::AngleMod;
using math_util::Sign;
using std::isfinite;
using std::string;
using std::vector;

namespace omnidrive {

CONFIG_FLOAT(max_accel, "co_max_accel");
CONFIG_FLOAT(max_angle_accel, "co_max_angle_accel");
CONFIG_FLOAT(max_speed, "co_max_speed");
CONFIG_FLOAT(max_angle_vel, "co_max_angle_vel");

OmnidirectionalModel::OmnidirectionalModel(const vector<string>& config_files) : RobotModel(),
                                                                                 angular_error_(0, 1),
                                                                                 config_reader_(config_files) {
    // ROS subscription initialized in Init()
}

void OmnidirectionalModel::DriveCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (!isfinite(msg->linear.x) || !isfinite(msg->linear.y) || !isfinite(msg->angular.z)) {
        printf("Ignoring non-finite drive values: linear.x=%f, linear.y=%f, angular.z=%f\n",
               msg->linear.x, msg->linear.y, msg->angular.z);
        return;
    }
    last_cmd_ = *msg;
    t_last_cmd_ = node_->now().seconds();
}

// TODO(jaholtz) Add noise
void OmnidirectionalModel::Step(const double& dt) {
    // Check command timeout
    static const double kMaxCommandAge = 0.1;
    if (IsCommandTimedOut(node_->now().seconds(), kMaxCommandAge)) {
        last_cmd_.linear.x = 0;
        last_cmd_.linear.y = 0;
        last_cmd_.angular.z = 0;
    }

    // Cap Velocity to max speed
    Vector2f desired_vel(last_cmd_.linear.x, last_cmd_.linear.y);
    if (desired_vel.norm() > CONFIG_max_speed) {
        desired_vel = CONFIG_max_speed * desired_vel.normalized();
    }

    // Cap acceleration to max accel
    Vector2f delta_v = desired_vel - vel_.translation;
    const float max_accel = CONFIG_max_accel * dt;
    if (delta_v.norm() > max_accel) {
        delta_v = max_accel * delta_v.normalized();
    }

    // Update translational velocity
    vel_.translation += delta_v;

    // Cap the rotational velocity and acceleration
    float desired_ang_vel = last_cmd_.angular.z;
    if (fabs(desired_ang_vel) > CONFIG_max_angle_vel) {
        desired_ang_vel = Sign(desired_ang_vel) * CONFIG_max_angle_vel;
    }
    const float max_angle_accel = CONFIG_max_angle_accel * dt;
    float delta_ang_v = desired_ang_vel - vel_.angle;
    if (fabs(desired_ang_vel) > max_angle_accel) {
        delta_ang_v = Sign(delta_ang_v) * max_angle_accel;
    }
    vel_.angle += delta_ang_v;

    pose_.translation += Rotation2Df(pose_.angle) * vel_.translation * dt;
    pose_.angle = AngleMod(pose_.angle + vel_.angle * dt);
    // Odometry publishing handled centrally by simulator
}

}  // namespace omnidrive
