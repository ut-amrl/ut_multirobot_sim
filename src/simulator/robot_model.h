//========================================================================
//      This software is free: you can redistribute it and/or modif    y
//  it under the terms of the GNU Lesser General Public License Vers    ion 3,
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
  \file    robot_model.h
  \brief   C++ Interface: Abstract class for robots built on entity_base
  \author  Jarrett Holtz, (C) 2020
  \email   jaholtz@cs.utexas.edu
*/
//========================================================================

#include "simulator/entity_base.h"

#ifndef SRC_SIMULATOR_ROBOT_MODEL_H_
#define SRC_SIMULATOR_ROBOT_MODEL_H_

namespace robot_model {
class RobotModel : public EntityBase {
 protected:
  Pose2Df vel_;

 public:
  RobotModel();
  virtual ~RobotModel() = default;
  virtual void SetVel(const pose_2d::Pose2Df& vel);
  virtual pose_2d::Pose2Df GetVel();
};
}  // namespace robot_model

#endif  // SRC_SIMULATOR_ROBOT_MODEL_H_
