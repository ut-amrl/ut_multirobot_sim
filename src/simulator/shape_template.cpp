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

ShapeTemplate::ShapeTemplate(){
  pose_ = Eigen::Vector3f(0., 0., 0.);
}

ShapeTemplate::~ShapeTemplate(){
  
}

void ShapeTemplate::step(double dt){

}

void ShapeTemplate::transform(){

}

Eigen::Vector3f ShapeTemplate::getGroundTruthPose(){
  return pose_;
}

std::vector<geometry::line2f> ShapeTemplate::getGroundTruthTemplate(){
  return template_lines_;
}
