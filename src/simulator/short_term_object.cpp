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
  \file    short_term_object.cpp
  \brief   C++ Interface: Short term object class
  \author  Yifeng Zhu, (C) 2020
  \email   yifeng.zhu@utexas.edu
*/
//========================================================================

#include "simulator/short_term_object.h"

ShortTermObject::ShortTermObject() {
  // angle, (x, y)
  pose_ = Pose2Df(0., Eigen::Vector2f(0., 0.));

  // Example of a simple shape
  const float r = 0.5;
  const float eps = 0.001;
  template_lines_.push_back(geometry::Line2f(Eigen::Vector2f(r - eps, r), Eigen::Vector2f(-r + eps, r)));
  template_lines_.push_back(geometry::Line2f(Eigen::Vector2f(-r, r - eps), Eigen::Vector2f(-r, -r + eps)));
  template_lines_.push_back(geometry::Line2f(Eigen::Vector2f(-r + eps, -r), Eigen::Vector2f(r - eps, -r)));
  template_lines_.push_back(geometry::Line2f(Eigen::Vector2f(r, -r + eps), Eigen::Vector2f(r, r - eps)));
  pose_lines_ = template_lines_;
}

ShortTermObject::ShortTermObject(const std::string& config_file) {
  pose_ = Pose2Df(0., Eigen::Vector2f(0., 0.));
  // TODO(yifeng) replace manually defined in the code with loading initial locations

  potential_initial_locs.push_back(Pose2Df(0., Eigen::Vector2f(-15., 8.6)));
  potential_initial_locs.push_back(Pose2Df(0., Eigen::Vector2f(-24., 8.6)));

  if (potential_initial_locs.size() > 0) {
    srand(time(0));
    const int rand_loc_idx = rand() % potential_initial_locs.size();
    pose_ = potential_initial_locs[rand_loc_idx];
  }

  // TODO(yifeng): Load the shape from a config file, replace the example in the future
  const double r = 0.5;
  const double eps = 0.001;
  template_lines_.push_back(geometry::Line2f(Eigen::Vector2f(r - eps, r), Eigen::Vector2f(-r + eps, r)));
  template_lines_.push_back(geometry::Line2f(Eigen::Vector2f(-r, r - eps), Eigen::Vector2f(-r, -r + eps)));
  template_lines_.push_back(geometry::Line2f(Eigen::Vector2f(-r + eps, -r), Eigen::Vector2f(r - eps, -r)));
  template_lines_.push_back(geometry::Line2f(Eigen::Vector2f(r, -r + eps), Eigen::Vector2f(r, r - eps)));

  Eigen::Rotation2Df R(math_util::AngleMod(pose_.angle));
  Eigen::Vector2f T = pose_.translation;
  pose_lines_.resize(template_lines_.size());
  for (size_t i=0; i < template_lines_.size(); i++) {
    pose_lines_[i].p0 = R * (template_lines_[i].p0) + T;
    pose_lines_[i].p1 = R * (template_lines_[i].p1) + T;
  }  
}



ShortTermObject::~ShortTermObject() {
}

