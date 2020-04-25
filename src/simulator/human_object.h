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
  \file    human_object.h
  \brief   C++ Interface: Human Object
  \author  Yifeng Zhu, (C) 2020
  \email   yifeng.zhu@utexas.edu
*/
//========================================================================

#include "simulator/entity_base.h"
#include "config_reader/config_reader.h"
#include "ros/publisher.h"
#include "ros/ros.h"
#include <string>
#ifndef SRC_SIMULATOR_HUMAN_OBJECT_H_
#define SRC_SIMULATOR_HUMAN_OBJECT_H_

using pose_2d::Pose2Df;

namespace human{

enum HumanMode{
     Singleshot,
     Repeat
};

class HumanObject: public EntityBase{
 protected:
  Pose2Df start_pose_;
  // TODO(yifeng): Change to a sequence of intermediate goals
  Pose2Df goal_pose_;
 
  // (vx, vy, vtheta)  
  Eigen::Vector2f trans_vel_;
  double rot_vel_;
  double max_speed_;
  double avg_speed_;
  double max_omega_;
  double avg_omega_;
  HumanMode mode_;
  double reach_goal_threshold_;

  config_reader::ConfigReader config_reader_;
  // (future) predefined trajectory if needed
  // bool use_predefined_traj=false;
  // double predefined_traj_freq;
  // std::vector<Eigen::Vector3f> predefined_traj;
 public:
  // Initialize a default object, probably a simple cylinder?
  HumanObject() = delete;
  // Intialize a default object reading from a file
  HumanObject(const std::vector<std::string>& config_file);
  ~HumanObject() = default;

  void SetGoalPose(const Pose2Df& goal_pose);
  // define step function for human object
  void Step(const double& dt);
  // transform lines from template lines based on pose_
  void Transform();

  // check if human reaches the current goal
  bool CheckReachGoal();
  // set the maximum speed for human
  void SetSpeed(const double& max_speed, const double& avg_speed, const double& max_omega = 0.4, const double& avg_omega = 0.2);
  void SetPose(const Pose2Df& pose);
  void SetVel(const Eigen::Vector2f& trans_vel, const double& rot_vel);
  void SetMode(const HumanMode& mode);
  double GetMaxSpeed();
  double GetAvgSpeed();
};

}  // namespace human

#endif  // SRC_SIMULATOR_HUMAN_OBJECT_H_
