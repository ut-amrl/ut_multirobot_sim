#include "door.h"

#include <cmath>
#include <eigen3/Eigen/Eigen>
#include <ros/ros.h>
#include <stdexcept>
#include <string>
#include <vector>

#include "config_reader/config_reader.h"
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"

using Eigen::Matrix2f;
using Eigen::Vector2f;
using geometry::Line2f;
using math_util::AngleDiff;
using math_util::Sign;
using std::invalid_argument;
using std::string;
using std::vector;

float normalize_angle(float angle) {
  angle = fmod(angle, M_2PI);
  if (angle < 0)
    angle += M_2PI;
  return angle;
}

namespace door {

CONFIG_FLOAT(length, "door_length");
CONFIG_VECTOR2F(axis_position, "door_axis_position");
CONFIG_FLOAT(open_angle, "door_open_angle");
CONFIG_FLOAT(closed_angle, "door_closed_angle");

Door::Door(const vector<string>& config_file) :
    EntityBase(DOOR),
    config_reader_(config_file) {
  length_ = CONFIG_length;
  axis_position_ = CONFIG_axis_position;
  open_angle_ = normalize_angle(CONFIG_open_angle);
  closed_angle_ = normalize_angle(CONFIG_closed_angle);
  angle_ = closed_angle_;
  state_ = CLOSED;

  // Check that the length of the line is legitimate.
  if (length_ <= 0.0) {
    throw invalid_argument("door length must be positive");
  }

  UpdateROS();

  // Set template_lines_ once, never to be touched again.
  // Does anyone else use/implement this? As far as I can tell it's just humans
  const Vector2f t_point1 = {0, 0};
  const Vector2f t_point2 = {length_, 0};
  const Line2f t_door_line(t_point1, t_point2);
  template_lines_.push_back(t_door_line);

  Transform();
}

void Door::UpdateROS() const {
  ros::param::set("/door_state", state_);
}

void Door::Transform() {
  pose_lines_.clear();
  const Vector2f p_point1 = axis_position_;
  const Vector2f p_point2 = axis_position_ + length_ * Vector2f(cos(angle_), sin(angle_));
  const Line2f p_door_line(p_point1, p_point2);
  pose_lines_.push_back(p_door_line);
}

void Door::Open() {
  SetState(OPENING);
}

void Door::Close() {
  SetState(CLOSING);
}

void Door::SetState(DoorState state) {
  state_ = state;
  UpdateROS();
}

void Door::Step(const double& dt) {
  if (state_ == OPENING) {
    if (abs(AngleDiff(angle_, open_angle_)) < 0.03) {
      angle_ = open_angle_;
      SetState(OPEN);
    } else {
      angle_ += 0.005 * Sign<float>(closed_angle_ - open_angle_);
    }
  } else if (state_ == CLOSING) {
    if (abs(AngleDiff(angle_, closed_angle_)) < 0.03) {
      angle_ = closed_angle_;
      SetState(CLOSED);
    } else {
      angle_ -= 0.005 * Sign<float>(closed_angle_ - open_angle_);
    }
  }
  angle_ = normalize_angle(angle_);
  Transform();
}

void Door::SetPose(const Pose2Df& pose) {
  //axis_position_ = pose.translation;
  ROS_INFO("SetPose");
  Transform();
}

Pose2Df Door::GetPose() {
  return Pose2Df(angle_, axis_position_);
}

vector<Line2f> Door::GetLines() {
  return pose_lines_;
}

vector<Line2f> Door::GetTemplateLines() {
  return template_lines_;
}

float Door::GetLength() const {
  return length_;
}

Vector2f Door::GetAxisPosition() const {
  return axis_position_;
}

float Door::GetOpenAngle() const {
  return open_angle_;
}

float Door::GetClosedAngle() const {
  return closed_angle_;
}

float Door::GetAngle() const {
  return angle_;
}

DoorState Door::GetState() const {
  return state_;
}

} // namespace door