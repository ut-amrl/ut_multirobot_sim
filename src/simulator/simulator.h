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
\author  Joydeep Biswas, (C) 2011
*/
//========================================================================

#include <iostream>
#include <stdio.h>
#include <vector>
#include "shared/util/proghelp.h"
#include "shared/util/timer.h"
#include "shared/math/geometry.h"
#include "map/vector_map.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include "f1tenth_simulator/AckermannDriveMsg.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/transform_broadcaster.h>

#ifndef SIMULATOR_H
#define SIMULATOR_H

using namespace std;

class AccelLimits{
  public:
    double max_accel;  // acceleration limit from 0 to max_vel
    double max_deccel; // acceleration limit from max_vel to 0
    double max_vel;    // maximum velocity along dimension

  public:
    void set(double a,double d,double v)
    {max_accel=a; max_deccel=d; max_vel=v;}

    // return new limits with all parameters scaled by <f>
    AccelLimits operator*(double f) const
    {AccelLimits r; r.set(max_accel*f,max_deccel*f,max_vel*f); return(r);}

    // scale all parameters by <f> in-place
    AccelLimits &operator*=(double f);

    // set limits to <al> with all parameters scaled by <f>
    AccelLimits &set(const AccelLimits &al,double f);
};

class Simulator{
  vector2d loc;
  double vel;
  double angVel;
  AccelLimits transLimits, rotLimits;

  ros::Subscriber driveSubscriber;
  ros::Subscriber initSubscriber;

  ros::Publisher odometryTwistPublisher;
  ros::Publisher laserPublisher;
  ros::Publisher mapLinesPublisher;
  ros::Publisher posMarkerPublisher;
  ros::Publisher dirMarkerPublisher;
  tf::TransformBroadcaster *br;

  // wheel orientations
  vector2d w0,w1,w2,w3;
  // Radius of base
  static const double baseRadius;
  // "height" of robot
  static const double robotHeight;

  sensor_msgs::LaserScan scanDataMsg;
  nav_msgs::Odometry odometryTwistMsg;

  VectorMap* currentMap;
  vector<VectorMap> maps;
  int curMapIdx;

  visualization_msgs::Marker lineListMarker;
  visualization_msgs::Marker robotPosMarker;
  visualization_msgs::Marker robotDirMarker;

  static const float startX;
  static const float startY;
  vector2f curLoc;

  static const float startAngle;
  float curAngle;

  double tLastCmd;

  static const float DT;
  static const float kMinR;

  f1tenth_simulator::AckermannDriveMsg last_cmd_;

private:
  void initVizMarker(visualization_msgs::Marker& vizMarker, string ns, int id,
string type, geometry_msgs::PoseStamped p, geometry_msgs::Point32 scale, double
duration, vector<float> color);
  void initSimulatorVizMarkers();
  void loadAtlas();
  void InitalLocationCallback(
      const geometry_msgs::PoseWithCovarianceStamped& msg);
  void AckermannDriveCallback(
      const f1tenth_simulator::AckermannDriveMsgConstPtr& msg);
  void publishOdometry();
  void publishLaser();
  void publishVisualizationMarkers();
  void publishTransform();
  void update();

public:
  Simulator();
  ~Simulator();
  void setLimits(AccelLimits _transLimits, AccelLimits _rotLimits){transLimits =
_transLimits; rotLimits = _rotLimits;}
  void init(ros::NodeHandle &n);
  void run();
};
#endif //SIMULATOR_H
