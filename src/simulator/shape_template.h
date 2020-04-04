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

#include <iostream>
#include <vector>
#include "eigen3/Eigen/Dense"
#include "math/line2d.h"

#ifndef SHAPE_TEMPLATE_H
#define SHAPE_TEMPLATE_H

class ShapeTemplate{
private:
    Eigen::Vector3f pose_; // (x, y, theta)
    std::vector<geometry::line2f> template_lines_;

public:
    ShapeTemplate();
    ~ShapeTemplate();
    // simulate a step for the object
    virtual void step(double dt);
    // transform the template to be placed at (x,y) with pose theta
    void transform();
    // get current ground truth pose of the obstacle
    virtual Eigen::Vector3f getGroundTruthPose();
    // get current ground truth shape (lines)
    virtual std::vector<geometry::line2f> getGroundTruthTemplate();
    
};

#endif // SHAPE_TEMPLATE_H
