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
\file    laser_scanner_main.cpp
\brief   The Laser Scanner module for CoBot II
\author  Joydeep Biswas, (C) 2010
*/
//========================================================================

#include <iostream>
#include <stdio.h>
#include "laser_scanner.h"
#include "terminal_utils.h"
#include "proghelp.h"
#include "sys/time.h"
#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include <tf/transform_broadcaster.h>
#include "configreader.h"
#include <ros/package.h>

#include "popt_pp.h"

bool run = true;
LaserScanner laserScanner;
const int numScanRays = 768;
double scanData[numScanRays];
sensor_msgs::LaserScan scanDataMsg;
ros::Publisher publisher;
tf::TransformBroadcaster *transformBroadcaster;

vector2f laserToBaseTrans;

void LoadParameters()
{
  WatchFiles watch_files;
  ConfigReader config(ros::package::getPath("cobot_linux").append("/").c_str());
  
  config.init(watch_files);
  
  config.addFile("../robot.cfg");
  config.addFile("config/localization_parameters.cfg");
  
  if(!config.readFiles()){
    printf("Failed to read config\n");
    exit(1);
  }
  
  {
    ConfigReader::SubTree c(config,"lidarParams");
    
    bool error = false;
    
    // Pose of laser sensor on robot
    error = error || !c.getVec2f<vector2f>("laserLoc", laserToBaseTrans);
    
    if(error){
      printf("Error Loading Lidar Parameters!\n");
      exit(2);
    }
  }
}

void timerEvent( int sig ) {
  const bool debugTimer = false;
  static const bool debug = false;
  static double tLast = GetTimeSec();
  if(debugTimer){
    printf( "dT = %f\n", GetTimeSec()-tLast );
    tLast = GetTimeSec();
  }
  
  laserScanner.run();
  if(laserScanner.getScanData(scanData)>0){
    for(int i=0;i<numScanRays;i++){
      scanDataMsg.ranges[i] = scanData[i];
    }
    scanDataMsg.header.seq++;
    scanDataMsg.header.stamp = ros::Time::now();
    publisher.publish(scanDataMsg);
    if(debug){
      printf( "dT = %f ", GetTimeSec()-tLast );
      tLast = GetTimeSec();
      printf("Range Center:");
      for(int i=382;i<386;i++)
        printf(" %f",scanData[i]);
      printf("\n");
    }
    
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(V2COMP(laserToBaseTrans), 0.23));
    transform.setRotation(tf::Quaternion(0, 0, 0, 1));
    transformBroadcaster->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "base_laser"));
  }else{
    if(debug) printf("No Laser data!\n");
  }
  if( !run ){
    CancelTimerInterrupts();
  }
}

int main(int argc, char **argv) {
  static const bool debug = false;
  if (debug) {
    ColourTerminal(TerminalUtils::TERMINAL_COL_WHITE,TerminalUtils::TERMINAL_COL_BLACK,TerminalUtils::TERMINAL_ATTR_BRIGHT);
		printf("\nCoBot Laser Scanner module\n\n");
    ResetTerminal();
	}
  
  int portNum = 0;
  // option table
  static struct poptOption options[] = {
    { "port-num", 'p', POPT_ARG_INT , &portNum,  0, "Laser Scanner serial port number", "NUM"},
    
    POPT_AUTOHELP
    { NULL, 0, 0, NULL, 0, NULL, NULL }
  };
  // parse options
  POpt popt(NULL,argc,(const char**)argv,options,0);
  int c;
  while((c = popt.getNextOpt()) >= 0){
  }
  
  LoadParameters();

  char serialPort[256];
  sprintf(serialPort,"/dev/ttyACM%d",portNum);
  if(debug) printf("Using port %s\n",serialPort);
  laserScanner.init(serialPort);
  
  ros::init(argc, argv, "Cobot_Laser_Module");
  ros::NodeHandle n;
  scanDataMsg.header.seq = 0;
  scanDataMsg.header.frame_id = "base_laser";
  scanDataMsg.angle_min = RAD(-135.0);
  scanDataMsg.angle_max = RAD(135.0);
  scanDataMsg.range_min = 0.02;
  scanDataMsg.range_max = 4.0;
  scanDataMsg.angle_increment = RAD(0.3515625);
  scanDataMsg.intensities.clear();
  scanDataMsg.ranges.resize(numScanRays);
  scanDataMsg.time_increment = 0.0;
  scanDataMsg.scan_time = 0.1;
  publisher = n.advertise<sensor_msgs::LaserScan>("Cobot/Laser", 1);
  transformBroadcaster = new tf::TransformBroadcaster();  
  // main program initialization
  InitHandleStop(&run);
  //Interrupt frequency of 50 Hz
  if(!SetTimerInterrupt(20000, &timerEvent)){
    TerminalWarning( "Unable to set timer interrupt\n" );
    laserScanner.close();
    return(1);
  }
  
  // main loop
  while(ros::ok() && run){
    ros::spinOnce();
    Sleep(0.01);
  }
  printf("closing.\n");
  laserScanner.close();
  return(0);
}

