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
 * \file    simulator_main.cpp
 * \brief   A simple simulator.
 * \author  Joydeep Biswas, (C) 2010
 */
//========================================================================

#include <stdio.h>

#include <iostream>

#include "gflags/gflags.h"
#include "ros/ros.h"

#include "shared/util/proghelp.h"
#include "shared/util/timer.h"
#include "simulator.h"

bool run = true;
Simulator simulator_;

void timerEvent(int sig) {
  const bool debugTimer = false;
  static double tLast = GetTimeSec();
  if(debugTimer) {
    printf( "dT = %f\n", GetTimeSec()-tLast );
    tLast = GetTimeSec();
  }
  simulator_.run();

  if( !run ) {
    CancelTimerInterrupts();
  }
}

int main(int argc, char **argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);
  printf("\nF1/10 Simulator\n\n");

  InitHandleStop(&run);
  ros::init(argc, argv, "F1Tenth_Simulator");
  ros::NodeHandle n;
  simulator_.init(n);

  //Interrupt frequency of 40 Hz
  if(!SetTimerInterrupt(25000, &timerEvent)) {
    printf("Unable to set timer interrupt\n");
    return(1);
  }

  // main loop
  while(ros::ok() && run){
    ros::spinOnce();
    Sleep(0.05);
  }

  printf("closing.\n");
  CancelTimerInterrupts();

  return(0);
}
