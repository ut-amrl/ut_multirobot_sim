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
  \file    short_term_object.h
  \brief   C++ Interface: Short term object class
  \author  Yifeng Zhu, (C) 2020
  \email   yifeng.zhu@utexas.edu
*/
//========================================================================

#include <string>
#include <vector>
#include "stdlib.h"
#include "time.h"
#include "simulator/entity_base.h"

#ifndef SRC_SIMULATOR_SHORT_TERM_OBJECT_H_
#define SRC_SIMULATOR_SHORT_TERM_OBJECT_H_

using pose_2d::Pose2Df;

class ShortTermObject: public EntityBase{
 protected:
  std::vector<Pose2Df> potential_initial_locs;
 public:
  // Initialize a default object, probably a simple cylinder?
  ShortTermObject();
  // Intialize a default object reading from a file
  explicit ShortTermObject(const std::string& config_file);
  ~ShortTermObject();
};


#endif  // SRC_SIMULATOR_SHORT_TERM_OBJECT_H_
