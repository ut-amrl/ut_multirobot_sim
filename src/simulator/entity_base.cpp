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

void EntityBase::Step(const double& dt) {
}

void EntityBase::SetPose(const Pose2Df& pose) {
  pose_ = pose;
}

Pose2Df EntityBase::GetPose() {
  return pose_;
}

std::vector<geometry::Line2f> EntityBase::GetTemplateLines() {
  return template_lines_;
}

std::vector<geometry::Line2f> EntityBase::GetLines() {
  return pose_lines_;
}
