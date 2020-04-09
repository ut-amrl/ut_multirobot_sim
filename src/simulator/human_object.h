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
#include <string>
#ifndef SRC_SIMULATOR_HUMAN_OBJECT_H_
#define SRC_SIMULATOR_HUMAN_OBJECT_H_

enum HumanMode{
     Singleshot,
     Repeat
};

class HumanObject: public EntityBase{
 protected:
  pose_2d::Pose2Df start_pose_;
  // TODO(yifeng): Change to a sequence of intermediate goals
  pose_2d::Pose2Df goal_pose_;
  double max_speed_;
  double avg_speed_;
  double max_omega_;
  double avg_omega_;
  bool mode_;
  double reach_goal_threashold_;
  // (future) predefined trajectory if needed
  // bool use_predefined_traj=false;
  // double predefined_traj_freq;
  // std::vector<Eigen::Vector3f> predefined_traj;
 public:
  // Initialize a default object, probably a simple cylinder?
  HumanObject();
  // Intialize a default object reading from a file
  explicit HumanObject(const std::string& config_file);
  ~HumanObject();

  // set start pose
  void initialize();
  void setGoalPose(const pose_2d::Pose2Df& goal_pose);
  // define step function for human object
  void step(const double& dt);

  // check if human reaches the current goal
  bool checkReachGoal();
  // set the maximum speed for human
  void setSpeed(const double& max_speed, const double& avg_speed, const double& max_omega = 0.4, const double& avg_omega = 0.2);
  void setGroundTruthPose(pose_2d::Pose2Df pose);
  void setGroundTruthVel(Eigen::Vector3f vel);
  void setMode(const HumanMode& mode);
  double getMaxSpeed();
  double getAvgSpeed();
};


#endif  // SRC_SIMULATOR_HUMAN_OBJECT_H_
