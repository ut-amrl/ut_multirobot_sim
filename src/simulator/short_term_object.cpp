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

#include "short_term_object.h"

ShortTermObject::ShortTermObject(){

  // x, y, angle
  pose_ = Eigen::Vector3f(0., 0., 0.);

  // Example of a simple shape
  double r = 0.5;
  double eps = 0.001;
  template_lines_.push_back(geometry::line2f(Eigen::Vector2f(r - eps, r), Eigen::Vector2f(-r + eps, r)));
  template_lines_.push_back(geometry::line2f(Eigen::Vector2f(-r, r - eps), Eigen::Vector2f(-r, -r + eps)));
  template_lines_.push_back(geometry::line2f(Eigen::Vector2f(-r + eps, -r), Eigen::Vector2f(r - eps, -r)));
  template_lines_.push_back(geometry::line2f(Eigen::Vector2f(r, -r + eps), Eigen::Vector2f(r, r - eps)));

  pose_lines_ = template_lines_;
}

ShortTermObject::ShortTermObject(std::string config_file){
  pose_ = Eigen::Vector3f(0., 0., 0.);
  // TODO replace manually defined in the code with loading initial locations

  potential_initial_locs.push_back(Eigen::Vector3f(-15., 8.6, 0.));
  potential_initial_locs.push_back(Eigen::Vector3f(-24., 8.6, 0.));

  if (potential_initial_locs.size() > 0){
    srand(time(0));
    int rand_loc_idx = rand() % potential_initial_locs.size();
    pose_ = potential_initial_locs[rand_loc_idx];
    std::cout << "Initialize with: " << rand_loc_idx << std::endl;
  }

  // TODO: Load the shape from a config file, replace the example in the future
  double r = 0.5;
  double eps = 0.001;
  template_lines_.push_back(geometry::line2f(Eigen::Vector2f(r - eps, r), Eigen::Vector2f(-r + eps, r)));
  template_lines_.push_back(geometry::line2f(Eigen::Vector2f(-r, r - eps), Eigen::Vector2f(-r, -r + eps)));
  template_lines_.push_back(geometry::line2f(Eigen::Vector2f(-r + eps, -r), Eigen::Vector2f(r - eps, -r)));
  template_lines_.push_back(geometry::line2f(Eigen::Vector2f(r, -r + eps), Eigen::Vector2f(r, r - eps)));

  pose_lines_ = template_lines_;
}



ShortTermObject::~ShortTermObject(){

}

