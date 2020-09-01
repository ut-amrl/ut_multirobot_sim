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

using namespace math_util;
using namespace ros_helpers;

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
    robot_vel_(0, 0),
    robot_omega_(0), */
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

size_t count = 0;

struct RobotCommand {
  RobotCommand(const Vector2f& location) : location(location), pause_time(0.0), type('l') {}
  RobotCommand(float pause_time) : location({0, 0}), pause_time(pause_time), type('p') {}

  //union {
    Vector2f location;
    float pause_time;
  //};

  char type;

};

queue<RobotCommand> commands({
  //RobotCommand(3.0),
  RobotCommand({30, 12.5}),
  RobotCommand({29, 10.06}),
  RobotCommand(2.0),
  RobotCommand({21.81, 9.11}),
  RobotCommand(0.7),
  RobotCommand({16, 9.11}),
  RobotCommand({15, 12})
});

enum DoorState {
  OPEN, CLOSED,
  OPENING, CLOSING
};

const Vector2f door_loc = {27.98, 9.62};

ofstream log;

void Navigation::Run() {

  /*
  if (!log.is_open()) {
    log.open("/home/simon/synth_data/" + random_string(20) + ".json");
  }
  */

  if (commands.empty()) {
    return;
  }

  int door_state;
  RobotCommand cmd = commands.front();
  if (cmd.type == 'l' && (odom_loc_ - cmd.location).norm() < 0.3) {
    commands.pop();
  } else if (cmd.type == 'p') {
    ros::Duration(cmd.pause_time).sleep();
    commands.pop();
  } else {
    const Vector2f current_waypoint = cmd.location;
    Vector2f robot_to_human = current_waypoint - odom_loc_;
    Vector2f robot_face = {cos(odom_angle_), sin(odom_angle_)};
    float dot = robot_to_human.x() * robot_face.x() + robot_to_human.y() * robot_face.y();
    float det = robot_to_human.x() * robot_face.y() - robot_to_human.y() * robot_face.x();
    float angle_diff = atan2(det, dot);
    float clamped_angle_diff = -Clamp<float>(angle_diff, -1, 1);

    ros::param::get("/door_state", door_state);

    if (cmd.location.x() == 16) {
      state_ = GOING_FORWARD_FAST;
    } else {
      state_ = GOING_FORWARD;
    }

    AckermannCurvatureDriveMsg msg;
    msg.curvature = clamped_angle_diff;
    msg.velocity = (state_ == STOPPED) ? 0 : (state_ == GOING_FORWARD_FAST ? 1.6 : 1);
    msg.header.stamp = ros::Time::now();
    drive_pub_.publish(msg);
  }

  if (false /*count++ % 20 == 8*/) {
    ROS_INFO("Writing example...");

    log
    << "  {\n"

    /*
    << "    \"human_pos_x\": {\n"
    << "      \"name\": \"human_pos_x\",\n"
    << "      \"value\": " << human_loc_.x() << ",\n"
    << "      \"type\": \"NUM\",\n"
    << "      \"dim\": [1, 0, 0]\n"
    << "    },\n"

    << "    \"human_pos_y\": {\n"
    << "      \"name\": \"human_pos_y\",\n"
    << "      \"value\": " << human_loc_.y() << ",\n"
    << "      \"type\": \"NUM\",\n"
    << "      \"dim\": [1, 0, 0]\n"
    << "    },\n"

    << "    \"robot_pos_x\": {\n"
    << "      \"name\": \"robot_pos_x\",\n"
    << "      \"value\": " << odom_loc_.x() << ",\n"
    << "      \"type\": \"NUM\",\n"
    << "      \"dim\": [1, 0, 0]\n"
    << "    },\n"

    << "    \"robot_pos_y\": {\n"
    << "      \"name\": \"robot_pos_y\",\n"
    << "      \"value\": " << odom_loc_.y() << ",\n"
    << "      \"type\": \"NUM\",\n"
    << "      \"dim\": [1, 0, 0]\n"
    << "    },\n"
    */
  
    << "    \"human_pos\": {\n"
    << "      \"name\": \"human_pos\",\n"
    << "      \"value\": [" << human_loc_.x() << ", " << human_loc_.y() << "],\n"
    << "      \"type\": \"VEC\",\n"
    << "      \"dim\": [1, 0, 0]\n"
    << "    },\n"

    << "    \"robot_pos\": {\n"
    << "      \"name\": \"robot_pos\",\n"
    << "      \"value\": [" << odom_loc_.x() << ", " << odom_loc_.y() << "],\n"
    << "      \"type\": \"VEC\",\n"
    << "      \"dim\": [1, 0, 0]\n"
    << "    },\n"

    << "    \"door_pos\": {\n"
    << "      \"name\": \"door_pos\",\n"
    << "      \"value\": [" << door_loc.x() << ", " << door_loc.y() << "],\n"
    << "      \"type\": \"VEC\",\n"
    << "      \"dim\": [1, 0, 0]\n"
    << "    },\n"

    << "    \"door_state\": {\n"
    << "      \"name\": \"door_state\",\n"
    << "      \"value\": " << (int) door_state << ",\n"
    << "      \"type\": \"NUM\",\n"
    << "      \"dim\": [0, 0, 0]\n"
    << "    },\n"

    << "    \"output\": {\n"
    << "      \"name\": \"output\",\n"
    << "      \"value\": " << (int) state_ << ",\n"
    << "      \"type\": \"NUM\",\n"
    << "      \"dim\": [1, 0, 0]\n"
    << "    }\n"

    << "  },\n";
  }

}

}  // namespace navigation
