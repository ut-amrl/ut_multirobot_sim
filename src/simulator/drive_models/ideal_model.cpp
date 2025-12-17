#include "simulator/drive_models/ideal_model.h"
#include "config_reader/config_reader.h"
#include <eigen3/Eigen/src/Geometry/Rotation2D.h>
#include "shared/util/timer.h"
#include "shared/math/math_util.h"
#include <geometry_msgs/msg/twist.hpp>

using Eigen::Rotation2Df;
using Eigen::Vector2f;
using geometry_msgs::msg::Twist;
using math_util::AngleMod;
using std::isfinite;
using std::string;

namespace ideal {

IdealModel::IdealModel(const std::string& config_file) : RobotModel() {
    // No configuration parameters needed for ideal model
    // Config file is accepted for API consistency but not used
}

void IdealModel::DriveCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (!isfinite(msg->linear.x) || !isfinite(msg->linear.y) || !isfinite(msg->angular.z)) {
        printf("Ignoring non-finite drive values: linear.x=%f, linear.y=%f, angular.z=%f\n",
               msg->linear.x, msg->linear.y, msg->angular.z);
        return;
    }
    last_cmd_ = *msg;
    t_last_cmd_ = node_->now().seconds();
}

void IdealModel::Step(const double& dt) {
    // Check command timeout (even ideal models should stop on timeout for safety)
    static const double kMaxCommandAge = 0.1;
    if (IsCommandTimedOut(node_->now().seconds(), kMaxCommandAge)) {
        last_cmd_.linear.x = 0;
        last_cmd_.linear.y = 0;
        last_cmd_.angular.z = 0;
    }

    // Directly apply commanded velocities without any limits
    vel_.translation.x() = last_cmd_.linear.x;
    vel_.translation.y() = last_cmd_.linear.y;
    vel_.angle = last_cmd_.angular.z;

    // Integrate pose: rotate translational velocity to world frame and apply
    pose_.translation += Rotation2Df(pose_.angle) * vel_.translation * dt;
    pose_.angle = AngleMod(pose_.angle + vel_.angle * dt);
}

}  // namespace ideal
