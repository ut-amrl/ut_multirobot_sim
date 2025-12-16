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
  \file    entity_base.h
  \brief   C++ Interface: Abstract class for objects
  \author  Yifeng Zhu, (C) 2020
  \email   yifeng.zhu@utexas.edu
*/
//========================================================================

#include <iostream>
#include <vector>
#include <cmath>
#include "eigen3/Eigen/Dense"
#include "shared/math/line2d.h"
#include "shared/math/poses_2d.h"
#ifndef SRC_SIMULATOR_ENTITY_BASE_H_
#define SRC_SIMULATOR_ENTITY_BASE_H_

using pose_2d::Pose2Df;

class EntityBase{
 protected:
    Pose2Df pose_;
    // template lines always assuming at pose (0., 0., 0.)
    std::vector<geometry::Line2f> template_lines_;
    // actual line position given current pose pose_
    std::vector<geometry::Line2f> pose_lines_;
 public:
    EntityBase();
    virtual ~EntityBase() = default;
    // simulate a step for the object
    virtual void Step(const double& dt);
    // set current pose
    virtual void SetPose(const Pose2Df& pose);
    // get current  pose of the obstacle
    virtual Pose2Df GetPose();
    // get current shape (lines) based on the pose
    virtual std::vector<geometry::Line2f> GetLines();
    // get template shape
    virtual std::vector<geometry::Line2f> GetTemplateLines();
};

#endif  // SRC_SIMULATOR_ENTITY_BASE_H_
