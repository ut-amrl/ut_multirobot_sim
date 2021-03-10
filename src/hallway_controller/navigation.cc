//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    navigation.cc
\brief   Starter code for navigation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "ut_multirobot_sim/AckermannCurvatureDriveMsg.h"
#include "ut_multirobot_sim/Pose2Df.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"

using Eigen::Vector2f;
using ut_multirobot_sim::AckermannCurvatureDriveMsg;
using std::ofstream;
using std::string;
using std::vector;

using namespace math_util;
using namespace ros_helpers;

DEFINE_double(Kp, -0.5, "proportional gain");
DEFINE_double(Kd, 15.0, "derivative gain");

namespace {
ros::Publisher drive_pub_;
ros::Publisher viz_pub_;
AckermannCurvatureDriveMsg drive_msg_;
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;
} //namespace

namespace navigation {

Navigation::Navigation(const string& map_file, ros::NodeHandle* n) :
    robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0),
    robot_omega_(0),
    nav_complete_(true),
    nav_goal_loc_(0, 0),
    nav_goal_angle_(0),
    state_(STOPPED) {
  drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>("ackermann_curvature_drive", 1);
  InitRosHeader("base_link", &drive_msg_.header);
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
  this->nav_goal_loc_ = loc;
  this->nav_goal_angle_ = angle;
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
  this->robot_loc_ = loc;
  this->robot_angle_ = angle;
}

void Navigation::UpdateOdometry(const Vector2f& loc,
                                float angle,
                                const Vector2f& vel,
                                float ang_vel) {
  this->odom_loc_ = loc;
  this->odom_angle_ = angle;
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud,
                                   double time) {
}

void Navigation::UpdateHuman(const Vector2f& loc, float angle, const Vector2f& trans_vel, float rot_vel) {
  this->human_loc_ = loc;
  this->human_angle_ = angle;
  this->human_vel_ = trans_vel;
  this->human_omega_ = rot_vel;
}

static float old_error = 0.0f;

float Navigation::PD(float target_y) {
  float error = odom_loc_.y() - target_y;
  float derivative = old_error - error;
  old_error = error;
  return Clamp(FLAGS_Kp * error + FLAGS_Kd * derivative, -1.0, 1.0);
}

void Navigation::GoingForward(AckermannCurvatureDriveMsg& msg) {
  msg.curvature = PD(odom_loc_.y());
  msg.velocity = 3.0f;
}

void Navigation::ShiftingLeft(AckermannCurvatureDriveMsg& msg) {
  msg.curvature = PD(22.5);
  msg.velocity = 3.0f;
}

void Navigation::ShiftingRight(AckermannCurvatureDriveMsg& msg) {
  msg.curvature = PD(21.5);
  msg.velocity = 3.0f;
}

void Navigation::Stopped(AckermannCurvatureDriveMsg& msg) {
  msg.velocity = 0.0f;
}

size_t count = 0;

void Navigation::Run() {
  
  float distance_ahead = human_loc_.x() - odom_loc_.x();

  if (distance_ahead >= 5.0) {
    state_ = GOING_FORWARD;
  } else if (distance_ahead < 5.0 && distance_ahead > 0.0) {
    state_ = SHIFTING_LEFT;
  } else {
    state_ = SHIFTING_RIGHT;
  }

  AckermannCurvatureDriveMsg msg;
  switch (state_) {
    case GOING_FORWARD:  this->GoingForward(msg);  break;
    case SHIFTING_LEFT:  this->ShiftingLeft(msg);  break;
    case SHIFTING_RIGHT: this->ShiftingRight(msg); break;
    case STOPPED:        this->Stopped(msg);       break;
    default:
      ROS_ERROR("Robot is in an invalid state!");
  }

  msg.header.stamp = ros::Time::now();
  drive_pub_.publish(msg);

}

}  // namespace navigation
