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
  \file    simulator.h
  \brief   C++ Interface: Simulator
  \author  Yifeng Zhu, (C) 2020
  \email   yifeng.zhu@utexas.edu
*/
//========================================================================

#include "shape_template.h"
#include <string>
#ifndef HUMAN_OBJECT_H
#define HUMAN_OBJECT_H

enum HumanMode{
	       Singleshot,
	       Repeat
};

class HumanObject: public ShapeTemplate{
  
protected:
  Eigen::Vector3f start_pose_;
  Eigen::Vector3f goal_pose_;
  double max_speed_;
  double avg_speed_;
  double max_omega_;
  double avg_omega_;
  bool mode_;
  bool present_;
public:
  // Initialize a default object, probably a simple cylinder?
  HumanObject();
  // Intialize a default object reading from a file
  HumanObject(std::string config_file);
  ~HumanObject();

  // set start pose
  void initialize();
  void setGoalPose(Eigen::Vector3f goal_pose);
  // define step function for human object
  void step(double dt);

  // check if human reaches the current goal
  bool checkReachGoal();
  // set the maximum speed for human
  void setSpeed(double max_speed, double avg_speed, double max_omega=0.4, double avg_omega=0.2);
  void setGroundTruthPose(Eigen::Vector3f pose);
  void setMode(HumanMode mode);
  double getMaxSpeed();
  double getAvgSpeed();
};


#endif // HUMAN_OBJECT_H
