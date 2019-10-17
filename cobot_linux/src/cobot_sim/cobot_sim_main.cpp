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
 * \file    cobot_sim_main.cpp
 * \brief   A simple Simulator for cobot
 * \author  Joydeep Biswas, (C) 2010
 */
//========================================================================

#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include "proghelp.h"
#include "timer.h"
#include "popt_pp.h"
#include "cobot_sim.h"

using namespace cobot_msgs;

bool run = true;
CobotSim *cobotSim;

void timerEvent( int sig ) {
  const bool debugTimer = false;
  static double tLast = GetTimeSec();
  if(debugTimer) {
    printf( "dT = %f\n", GetTimeSec()-tLast );
    tLast = GetTimeSec();
  }
  cobotSim->run();

  if( !run ) {
    CancelTimerInterrupts();
  }
}

int main(int argc, char **argv) {
  ColourTerminal(TerminalUtils::TERMINAL_COL_WHITE,TerminalUtils::TERMINAL_COL_BLACK,TerminalUtils::TERMINAL_ATTR_BRIGHT);
  printf("\nCoBot Simulator\n\n");
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

  cobotSim = new CobotSim();

  InitHandleStop(&run);
  AccelLimits transLimits, rotLimits;
  transLimits.set(0.5,0.5,0.5);
  rotLimits.set(2.0,2.0,1.5);
  cobotSim->setLimits(transLimits, rotLimits);

  ros::init(argc, argv, "Cobot_Simulator");
  ros::NodeHandle n;
  cobotSim->init(n);

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
  delete cobotSim;

  return(0);
}
