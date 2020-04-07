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
  \file    shape_template.cpp
  \brief   C++ Interface: Abstract class for objects
  \author  Yifeng Zhu, (C) 2020
  \email   yifeng.zhu@utexas.edu
*/
//========================================================================

#include "shape_template.h"

ShapeTemplate::ShapeTemplate(){
  
}

ShapeTemplate::~ShapeTemplate(){
  
}

double ShapeTemplate::wrapAngle(double angle){
  angle = fmod(angle + M_PI, 2 * M_PI);
  if (angle < 0){
    angle += 2 * M_PI;
   }
   angle -= M_PI;
  return angle;
}

void ShapeTemplate::step(double dt){
  // pose_[2] += 0.1 * dt;
  // pose_[0] += 0.03 * dt;
  // pose_[2] = this->wrapAngle(pose_[2]);

  // update the shape based on the new pose
  this->transform();
}

void ShapeTemplate::transform(){
  pose_[2] = this->wrapAngle(pose_[2]);
  Eigen::Rotation2Df R(pose_[2]);
  Eigen::Vector2f T(pose_[0], pose_[1]);

  for (size_t i=0; i < template_lines_.size(); i++){
    pose_lines_[i].p0 = R * (template_lines_[i].p0) + T;
    pose_lines_[i].p1 = R * (template_lines_[i].p1) + T;
  }
}

void ShapeTemplate::setGroundTruthPose(Eigen::Vector3f pose){
  pose_ = pose;
  // update the shape according to the new pose
  this->transform();
}

void ShapeTemplate::setGroundTruthVel(Eigen::Vector3f vel){
  vel_ = vel;
  // update the shape according to the new vel
}

Eigen::Vector3f ShapeTemplate::getGroundTruthPose(){
  return pose_;
}

std::vector<geometry::line2f> ShapeTemplate::getTemplateLines(){
  return template_lines_;
}

std::vector<geometry::line2f> ShapeTemplate::getGroundTruthLines(){
  return pose_lines_;
}
