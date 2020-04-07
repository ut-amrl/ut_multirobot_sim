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

#include "human_object.h"

HumanObject::HumanObject(){
  // x, y, angle
  pose_ = Eigen::Vector3f(0., 0., 0.);
  // just a cylinder for now
  double r = 0.3;
  int num_segments = 20;

  double angle_increment = 2 * M_PI / num_segments;

  Eigen::Vector2f v0(r, 0.);
  Eigen::Vector2f v1;
  double eps = 0.001;
  for (int i = 1; i < num_segments; i++){
    v1 = Eigen::Rotation2Df(angle_increment * i) * Eigen::Vector2f(r, 0.0);

    // TODO: Fix the vector map bug that closed shape would have wrong occlusion
    Eigen::Vector2f eps_vec = (v1 - v0).normalized() * eps;
    template_lines_.push_back(geometry::line2f(v0 + eps_vec, v1 - eps_vec));
    v0 = v1;
  }
  template_lines_.push_back(geometry::line2f(v1, Eigen::Vector2f(r, 0.0)));
  pose_lines_ = template_lines_;
  this->initialize();
}

HumanObject::HumanObject(std::string config_file){
  // TODO: Load the shape, start point, goal point and walking mode from a config file
  this->initialize();
}


HumanObject::~HumanObject(){

}

void HumanObject::initialize(){
  start_pose_ = pose_;
  present_ = true;
  mode_ = HumanMode::Singleshot;
  max_speed_ = 1.5;
  avg_speed_ = 1.0;
}

void HumanObject::setMode(HumanMode mode){
  mode_ = mode;
}

void HumanObject::setGoalPose(Eigen::Vector3f goal_pose){
  goal_pose_ = goal_pose;
}

void HumanObject::setGroundTruthPose(Eigen::Vector3f pose){
  pose_ = pose;
  // update the shape according to the new pose
  this->initialize();
  this->transform();
}


void HumanObject::step(double dt){
  // very simple dynamic update
  vel_.head<2>() = (goal_pose_.head<2>() - pose_.head<2>()).normalized() * avg_speed_;
  // TODO: Add gaussian noise to the velocity

  // clip velocity if it is larger than max speed
  if (vel_.head<2>().norm() > max_speed_){
    vel_.head<2>() = vel_.head<2>().normalized() * max_speed_;
  }
  vel_(2) = 0.1;
  pose_ = pose_ + vel_ * dt;
  this->transform();
  this->checkReachGoal();
}

void HumanObject::setSpeed(double max_speed, double avg_speed, double max_omega, double avg_omega){
  max_speed_ = max_speed;
  avg_speed_ = avg_speed;
  max_omega_ = max_omega;
  avg_omega_ = avg_omega;
}

double HumanObject::getMaxSpeed(){
  return max_speed_;
}

double HumanObject::getAvgSpeed(){
  return avg_speed_;
}

bool HumanObject::checkReachGoal(){
  if ((pose_.head<2>() - goal_pose_.head<2>()).norm() < 0.3){
    // reached goal
    if (mode_ == HumanMode::Singleshot){
      vel_.setZero();
	return true;
    }
    else if (mode_ == HumanMode::Repeat){
      Eigen::Vector3f temp = goal_pose_;
      goal_pose_ = start_pose_;
      start_pose_ = temp;
    }
  }
  return false;
}
