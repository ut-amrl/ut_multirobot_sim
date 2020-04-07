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

#include "shape_template.h"
#include <string>
#include "stdlib.h"
#include "time.h"

#ifndef SHORT_TERM_OBJECT_H
#define SHORT_TERM_OBJECT_H

class ShortTermObject: public ShapeTemplate{
protected:
  std::vector<Eigen::Vector3f> potential_initial_locs;
public:
  // Initialize a default object, probably a simple cylinder?
  ShortTermObject();
  // Intialize a default object reading from a file
  ShortTermObject(std::string config_file);
  ~ShortTermObject();
};


#endif // SHORT_TERM_OBJECT_H
