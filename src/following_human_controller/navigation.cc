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

#include <iostream>
#include <queue>

using namespace std;

using Eigen::Vector2f;
using ut_multirobot_sim::AckermannCurvatureDriveMsg;
using std::ofstream;
using std::queue;
using std::string;
using std::vector;

extern bool manual;

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

string random_string(size_t length) {

  static const char alphanum[] =
    "0123456789"
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz";

  srand(time(NULL));

  string ret;
  for (size_t i = 0; i < length; ++i)
    ret += alphanum[rand() % (sizeof(alphanum) - 1)];

  return ret;
}

namespace navigation {

Navigation::Navigation(const string& map_file, ros::NodeHandle* n) :
    /*robot_loc_(0, 0),
    robot_angle_(0),
    robot_vel_(0, 0), */
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
  this->odom_loc_ = loc;
  this->odom_angle_ = angle;
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

void Navigation::Run() {

  Vector2f robot_to_human = human_loc_ - odom_loc_;
  Vector2f robot_face = {cos(odom_angle_), sin(odom_angle_)};
  float dot = robot_to_human.x() * robot_face.x() + robot_to_human.x() * robot_face.y();
  float det = robot_to_human.x() * robot_face.y() - robot_to_human.y() * robot_face.x();
  float angle_diff = atan2(det, dot);
  float clamped_angle_diff = -Clamp<float>(angle_diff, -1, 1);

  if ((odom_loc_ - human_loc_).norm() < 1.0) {
    state_ = STOPPED;
  } else {
    state_ = GOING_FORWARD;
  }

  AckermannCurvatureDriveMsg msg;
  msg.curvature = clamped_angle_diff;
  msg.velocity = (state_ == STOPPED) ? 0.05 : 1;
  msg.header.stamp = ros::Time::now();
  drive_pub_.publish(msg);
}

}  // namespace navigation
