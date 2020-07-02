#include "simulator/ackermann_model.h"
#include "shared/util/timer.h"
#include "shared/math/math_util.h"
#include <eigen3/Eigen/src/Geometry/Rotation2D.h>
#include <string>

using Eigen::Vector2f;
using Eigen::Rotation2Df;
#ifdef AMRL_MSGS
  using amrl_msgs::AckermannCurvatureDriveMsg;
#else
  using ut_multirobot_sim::AckermannCurvatureDriveMsg;
#endif
using math_util::Bound;
using math_util::AngleDiff;
using math_util::AngleMod;
using std::isfinite;
using std::string;
using std::vector;

namespace ackermann {

CONFIG_FLOAT(min_turn_r, "ak_min_turn_radius");
CONFIG_FLOAT(max_accel, "ak_max_accel");
CONFIG_FLOAT(max_speed, "ak_max_speed");
CONFIG_FLOAT(angular_bias, "ak_angular_error_bias");
CONFIG_FLOAT(angular_error, "ak_angular_error_rate");
CONFIG_STRING(drive_topic, "ak_drive_callback_topic");
CONFIG_FLOAT(radius, "ackermann_radius");
CONFIG_FLOAT(num_segments, "ackermann_num_segments");
CONFIG_VECTOR2F(offset_vec, "ackermann_offset");


AckermannModel::AckermannModel(const vector<string>& config_file, ros::NodeHandle* n, string topic_prefix = "") :
    RobotModel(),
    last_cmd_(),
    t_last_cmd_(0),
    angular_error_(0, 1),
    config_reader_(config_file){
  // Use the config reader to initialize the subscriber
  last_cmd_.velocity = 0;
  last_cmd_.curvature = 0;
  drive_subscriber_ = n->subscribe(
      topic_prefix + CONFIG_drive_topic,
      1,
      &AckermannModel::DriveCallback,
      this);
  this->SetTemplateLines(CONFIG_radius, CONFIG_num_segments);
  this->Transform();
}

void AckermannModel::DriveCallback(const AckermannCurvatureDriveMsg& msg) {
  if (!isfinite(msg.velocity) || !isfinite(msg.curvature)) {
    printf("Ignoring non-finite drive values: %f %f\n",
        msg.velocity,
        msg.curvature);
    return;
  }
  last_cmd_ = msg;
  t_last_cmd_ = GetMonotonicTime();
}

void AckermannModel::Step(const double &dt) {
  // TODO(jaholtz) For faster than real time simulation we may need
  // a wallclock invariant method for this.
  static const double kMaxCommandAge = 0.1;
  if (GetMonotonicTime() > t_last_cmd_ + kMaxCommandAge) {
    last_cmd_.velocity = 0;
  }
  const float vel = vel_.translation.x();
  // Epsilon curvature corresponding to a very large radius of turning.
  static const float kEpsilonCurvature = 1.0 / 1E3;
  // Commanded speed bounded to motion limit.
  float desired_vel = last_cmd_.velocity;
  Bound(-CONFIG_max_speed, CONFIG_max_speed, &desired_vel);
  // Maximum magnitude of curvature according to turning limits.
  const float max_curvature = 1.0 / CONFIG_min_turn_r;
  // Commanded curvature bounded to turning limit.
  float desired_curvature = last_cmd_.curvature;
  Bound(-max_curvature, max_curvature, &desired_curvature);
  // Indicates if the command is for linear motion.
  const bool linear_motion = (fabs(desired_curvature) < kEpsilonCurvature);

  const float dv_max = dt * CONFIG_max_accel;
  float bounded_dv = desired_vel - vel;
  Bound(-dv_max, dv_max, &bounded_dv);
  // Set velocity
  vel_.translation.x() = vel + bounded_dv;
  const float dist = vel_.translation.x() * dt;

  Vector2f d_vector(0,0);
  float dtheta = 0;
  if (linear_motion) {
    d_vector.x() = dist;
    dtheta = dt * CONFIG_angular_bias;
  } else {
    const float r = 1.0 / desired_curvature;
    dtheta = dist * desired_curvature +
        angular_error_(rng_) * dt * CONFIG_angular_bias +
        angular_error_(rng_) * CONFIG_angular_error * fabs(dist *
            desired_curvature);
    d_vector = {r * sin(dtheta), r * (1.0 - cos(dtheta))};
  }
  // Update the Pose
  pose_.translation += Eigen::Rotation2Df(pose_.angle) * d_vector;
  pose_.angle = AngleMod(pose_.angle + dtheta);
  this->Transform();
}

void AckermannModel::Transform() {
  Eigen::Rotation2Df R(math_util::AngleMod(pose_.angle));
  Eigen::Vector2f T = pose_.translation;

  for (size_t i=0; i < template_lines_.size(); i++) {
    pose_lines_[i].p0 = R * (template_lines_[i].p0) + T;
    pose_lines_[i].p1 = R * (template_lines_[i].p1) + T;
  }
}

void AckermannModel::SetTemplateLines(const float r, const int num_segments){
  // copied directly from human model. In future refractor the code to 
  // enable inherentance might be more optimal. For now there is an excessive
  // amount of unwanted code in HumanModel, which disables inherenting directly.

  const float angle_increment = 2 * M_PI / num_segments;

  Eigen::Vector2f v0(r, 0.);
  Eigen::Vector2f v1;
  Eigen::Vector2f offset_vec = CONFIG_offset_vec;
  const float eps = 0.0005;
  for (int i = 1; i < num_segments; i++) {
    v1 = Eigen::Rotation2Df(angle_increment * i) * Eigen::Vector2f(r, 0.0);

    Eigen::Vector2f eps_vec = (v1 - v0).normalized() * eps;
    template_lines_.push_back(geometry::Line2f(v0 + eps_vec + offset_vec, v1 - eps_vec + offset_vec));
    v0 = v1;
  }
  template_lines_.push_back(geometry::Line2f(v1, Eigen::Vector2f(r, 0.0)));
  pose_lines_ = template_lines_;
}
} // namespace ackermann
