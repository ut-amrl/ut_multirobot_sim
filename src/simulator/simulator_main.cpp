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

#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include "shared/util/proghelp.h"
#include "shared/util/timer.h"
#include "shared/util/popt_pp.h"
#include "simulator.h"


bool run = true;
Simulator simulator_;

void timerEvent( int sig ) {
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

ColourTerminal(TerminalUtils::TERMINAL_COL_WHITE,TerminalUtils::
TERMINAL_COL_BLACK,TerminalUtils::TERMINAL_ATTR_BRIGHT);
  printf("\nF1/10 Simulator\n\n");
  ResetTerminal();

  // option table
  static struct poptOption options[] = {
    POPT_AUTOHELP
    { NULL, 0, 0, NULL, 0, NULL, NULL }
  };
  // parse options
  POpt popt(NULL,argc,(const char**)argv,options,0);
  int c;
  while((c = popt.getNextOpt()) >= 0) { }

  InitHandleStop(&run);
  AccelLimits transLimits, rotLimits;
  transLimits.set(0.5,0.5,0.5);
  rotLimits.set(2.0,2.0,1.5);
  simulator_.setLimits(transLimits, rotLimits);

  ros::init(argc, argv, "F1Tenth_Simulator");
  ros::NodeHandle n;
  simulator_.init(n);

  //Interrupt frequency of 20 Hz
  if(!SetTimerInterrupt(50000, &timerEvent)) {
    TerminalWarning( "Unable to set timer interrupt\n" );
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
