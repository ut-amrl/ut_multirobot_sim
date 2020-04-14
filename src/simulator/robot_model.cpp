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
  \file    robot_model.cpp
  \brief   C++ Interface: Abstract class for robot models
  \author  Jarrett Holtz, (C) 2020
  \email   jaholtz@cs.utexas.edu
*/
//========================================================================

#include "simulator/robot_model.h"

namespace robot_model {

RobotModel::RobotModel() :
    EntityBase(),
    vel_(0,{0,0}) {}

void RobotModel::SetVel(const pose_2d::Pose2Df& vel) {
 vel_ = vel;
}

Pose2Df RobotModel::GetVel() {
  return vel_;
}

}
