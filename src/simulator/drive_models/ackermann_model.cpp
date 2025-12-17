#include "simulator/drive_models/ackermann_model.h"
#include "shared/util/timer.h"
#include "shared/math/math_util.h"
#include <eigen3/Eigen/src/Geometry/Rotation2D.h>

using Eigen::Rotation2Df;
using Eigen::Vector2f;
using geometry_msgs::msg::Twist;
using math_util::AngleDiff;
using math_util::AngleMod;
using math_util::Bound;
using std::isfinite;
using std::string;
using std::vector;

namespace ackermann {

CONFIG_FLOAT(min_turn_r, "ak_min_turn_radius");
CONFIG_FLOAT(max_accel, "ak_max_accel");
CONFIG_FLOAT(max_speed, "ak_max_speed");
CONFIG_FLOAT(angular_bias, "ak_angular_error_bias");
CONFIG_FLOAT(angular_error, "ak_angular_error_rate");

AckermannModel::AckermannModel(const vector<string>& config_files) : RobotModel(),
                                                                     angular_error_(0, 1),
                                                                     config_reader_(config_files) {
    // ROS subscription initialized in Init()
}

void AckermannModel::DriveCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (!isfinite(msg->linear.x) || !isfinite(msg->angular.z)) {
        printf("Ignoring non-finite drive values: linear.x=%f, angular.z=%f\n",
               msg->linear.x, msg->angular.z);
        return;
    }
    last_cmd_ = *msg;
    t_last_cmd_ = node_->now().seconds();
}

void AckermannModel::Step(const double& dt) {
    // TODO(jaholtz) For faster than real time simulation we may need
    // a wallclock invariant method for this.
    static const double kMaxCommandAge = 0.1;
    if (IsCommandTimedOut(node_->now().seconds(), kMaxCommandAge)) {
        last_cmd_.linear.x = 0;
        last_cmd_.angular.z = 0;
    }

    const float vel = vel_.translation.x();
    // Epsilon curvature corresponding to a very large radius of turning.
    static const float kEpsilonCurvature = 1.0 / 1E3;
    // Commanded speed bounded to motion limit.
    float desired_vel = last_cmd_.linear.x;
    Bound(-CONFIG_max_speed, CONFIG_max_speed, &desired_vel);

    // Convert Twist (velocities) to Ackermann parameters (velocity + curvature)
    // curvature = angular_velocity / linear_velocity (when linear_velocity != 0)
    float desired_curvature = 0.0f;
    if (fabs(last_cmd_.linear.x) < 1e-6) {
        // If velocity is near zero, set curvature to zero (stop turning)
        desired_curvature = 0.0f;
    } else {
        desired_curvature = last_cmd_.angular.z / last_cmd_.linear.x;
    }

    // Maximum magnitude of curvature according to turning limits.
    const float max_curvature = 1.0 / CONFIG_min_turn_r;
    // Commanded curvature bounded to turning limit.
    Bound(-max_curvature, max_curvature, &desired_curvature);
    // Indicates if the command is for linear motion.
    const bool linear_motion = (fabs(desired_curvature) < kEpsilonCurvature);

    const float dv_max = dt * CONFIG_max_accel;
    float bounded_dv = desired_vel - vel;
    Bound(-dv_max, dv_max, &bounded_dv);
    // Set velocity
    vel_.translation.x() = vel + bounded_dv;
    const float dist = vel_.translation.x() * dt;

    Vector2f d_vector(0, 0);
    float dtheta = 0;
    if (linear_motion) {
        d_vector.x() = dist;
        dtheta = dt * CONFIG_angular_bias;
    } else {
        const float r = 1.0 / desired_curvature;
        dtheta = dist * desired_curvature +
                 angular_error_(rng_) * dt * CONFIG_angular_bias +
                 angular_error_(rng_) * CONFIG_angular_error * fabs(dist * desired_curvature);
        d_vector = {r * sin(dtheta), r * (1.0 - cos(dtheta))};
    }
    // Update the Pose
    pose_.translation += Eigen::Rotation2Df(pose_.angle) * d_vector;
    pose_.angle = AngleMod(pose_.angle + dtheta);
}

}  // namespace ackermann
