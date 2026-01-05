#include "simulator/drive_models/ackermann_model.h"

#include <cmath>
#include <cstdio>

#include "eigen3/Eigen/Geometry"
#include "config_reader/config_reader.h"
#include "shared/math/math_util.h"

using Eigen::Vector2f;
using math_util::AngleMod;
using math_util::Bound;

namespace ackermann {

AckermannModel::AckermannModel(const std::string& config_file) : RobotModel() {
    CONFIG_FLOAT(min_turn_radius, "min_turn_radius");
    CONFIG_FLOAT(max_accel, "max_accel");
    CONFIG_FLOAT(max_speed, "max_speed");
    CONFIG_FLOAT(turning_error_bias, "turning_error_bias");
    CONFIG_FLOAT(turning_error_rate, "turning_error_rate");

    // Load config from file
    config_reader::ConfigReader reader({config_file});

    min_turn_radius_ = CONFIG_min_turn_radius;
    max_accel_ = CONFIG_max_accel;
    max_speed_ = CONFIG_max_speed;
    turning_error_bias_ = CONFIG_turning_error_bias;
    turning_error_rate_ = CONFIG_turning_error_rate;

    turning_error_ = std::normal_distribution<float>(0.0f, turning_error_rate_);

    // ROS subscription initialized in Init()
}

void AckermannModel::DriveCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->angular.z)) {
        std::printf("Ignoring non-finite drive values: linear.x=%f, angular.z=%f\n",
                    msg->linear.x, msg->angular.z);
        return;
    }
    StoreCommandTimestamp(msg);
    last_cmd_ = *msg;
    new_cmd_received_ = true;
}

void AckermannModel::Step(const double& dt) {
    // Check command timeout (uses simulation time)
    if (IsCommandTimedOut()) {
        last_cmd_.linear.x = 0;
        last_cmd_.angular.z = 0;
    }

    const float vel = vel_.translation.x();
    // Epsilon curvature corresponding to a very large radius of turning.
    static const float kEpsilonCurvature = 1.0 / 1E3;
    // Commanded speed bounded to motion limit.
    float desired_vel = last_cmd_.linear.x;
    Bound(-max_speed_, max_speed_, &desired_vel);

    // Convert Twist (velocities) to Ackermann parameters (velocity + curvature)
    // curvature = angular_velocity / linear_velocity (when linear_velocity != 0)
    float desired_curvature = 0.0f;
    if (std::fabs(last_cmd_.linear.x) < 1e-6f) {
        // If velocity is near zero, set curvature to zero (stop turning)
        desired_curvature = 0.0f;
    } else {
        desired_curvature = last_cmd_.angular.z / last_cmd_.linear.x;
    }

    // Maximum magnitude of curvature according to turning limits.
    const float max_curvature = 1.0 / min_turn_radius_;
    // Commanded curvature bounded to turning limit.
    Bound(-max_curvature, max_curvature, &desired_curvature);
    // Indicates if the command is for linear motion.
    const bool linear_motion = (std::fabs(desired_curvature) < kEpsilonCurvature);

    const float dv_max = dt * max_accel_;
    float bounded_dv = desired_vel - vel;
    Bound(-dv_max, dv_max, &bounded_dv);
    // Set velocity
    vel_.translation.x() = vel + bounded_dv;
    const float dist = vel_.translation.x() * dt;

    Vector2f d_vector(0, 0);
    float dtheta = 0;
    if (linear_motion) {
        d_vector.x() = dist;
        dtheta = turning_error_bias_ * dt + turning_error_(rng_) * dt;
    } else {
        const float r = 1.0 / desired_curvature;
        const float base_dtheta = dist * desired_curvature;
        const float error_sample = turning_error_(rng_);
        dtheta = base_dtheta +
                 turning_error_bias_ * dt +
                 error_sample * std::fabs(base_dtheta);
        d_vector = {r * std::sin(dtheta), r * (1.0f - std::cos(dtheta))};
    }
    // Track angular velocity for downstream odometry publication
    vel_.angle = (dt > 0.0) ? dtheta / dt : 0.0f;
    // Update the Pose
    pose_.translation += Eigen::Rotation2Df(pose_.angle) * d_vector;
    pose_.angle = AngleMod(pose_.angle + dtheta);
}

}  // namespace ackermann
