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
  \file    entity_base.cpp
  \brief   C++ Interface: Abstract class for objects
  \author  Yifeng Zhu, (C) 2020
  \email   yifeng.zhu@utexas.edu
*/
//========================================================================

#include "simulator/entity_base.h"

EntityBase::EntityBase() {
}

EntityBase::~EntityBase() {
}

void EntityBase::Step(const double& dt) {
  // pose_[2] += 0.1 * dt;
  // pose_[0] += 0.03 * dt;

  // update the shape based on the new pose
  this->Transform();
}

void EntityBase::Transform() {
  Eigen::Rotation2Df R(math_util::AngleMod(pose_.angle));
  Eigen::Vector2f T = pose_.translation;

  for (size_t i=0; i < template_lines_.size(); i++) {
    pose_lines_[i].p0 = R * (template_lines_[i].p0) + T;
    pose_lines_[i].p1 = R * (template_lines_[i].p1) + T;
  }
}

void EntityBase::SetGroundTruthPose(const Pose2Df& pose) {
  pose_ = pose;
  // update the shape according to the new pose
  this->Transform();
}

Pose2Df EntityBase::GetGroundTruthPose() {
  return pose_;
}

std::vector<geometry::line2f> EntityBase::GetTemplateLines() {
  return template_lines_;
}

std::vector<geometry::line2f> EntityBase::GetLines() {
  return pose_lines_;
}
