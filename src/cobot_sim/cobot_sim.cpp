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
\file    cobot_sim.h
\brief   C++ Implementation: CobotSim
\author  Joydeep Biswas, (C) 2011
*/
//========================================================================


#include "cobot_sim.h"
#include <math.h>
#include "config_reader/config_reader.h"

#include "geometry_msgs/PoseWithCovarianceStamped.h"

using f1tenth_simulator::AckermanDriveMsgConstPtr;
using f1tenth_simulator::AckermanDriveMsg;
using geometry_msgs::PoseWithCovarianceStamped;

CONFIG_STRING(cMapName, "map_name");
CONFIG_FLOAT(cAxisLength, "axis_length");
CONFIG_FLOAT(cStartX, "start_x");
CONFIG_FLOAT(cStartY, "start_y");
CONFIG_FLOAT(cStartAngle, "start_angle");
CONFIG_FLOAT(cDT, "delta_t");
CONFIG_FLOAT(cMinTurnR, "min_turn_radius");
config_reader::ConfigReader reader({"config/f1_config.lua"});

const double CobotSim::robotHeight = 0.36;
// // const float CobotSim::startX = -7.5;
// // const float CobotSim::startY = 1.0;
// const float CobotSim::startAngle = 0.0;
// const float CobotSim::DT = 0.05;
// const float CobotSim::kMinR = 0.1;

CobotSim::CobotSim() {
  w0.heading(RAD(45.0));
  w1.heading(RAD(135.0));
  w2.heading(RAD(-135.0));
  w3.heading(RAD(-45.0));
  tLastCmd = GetTimeSec();
}

CobotSim::~CobotSim() { }

void CobotSim::init(ros::NodeHandle& n) {
  scanDataMsg.header.seq = 0;
  scanDataMsg.header.frame_id = "base_laser";
  scanDataMsg.angle_min = RAD(-135.0);
  scanDataMsg.angle_max = RAD(135.0);
  scanDataMsg.range_min = 0.02;
  scanDataMsg.range_max = 4.0;
  scanDataMsg.angle_increment = RAD(360.0)/1024.0;
  scanDataMsg.intensities.clear();
  scanDataMsg.time_increment = 0.0;
  scanDataMsg.scan_time = 0.05;

  odometryTwistMsg.header.seq = 0;
  odometryTwistMsg.header.frame_id = "odom";
  odometryTwistMsg.child_frame_id = "base_footprint";

  curLoc.set(cStartX, cStartY);
  curAngle = cStartAngle;

  initCobotSimVizMarkers();
  loadAtlas();

  driveSubscriber = n.subscribe(
      "/ackerman_drive", 1, &CobotSim::AckermanDriveCallback, this);
  initSubscriber = n.subscribe(
      "/initialpose", 1, &CobotSim::InitalLocationCallback, this);
  odometryTwistPublisher = n.advertise<nav_msgs::Odometry>("/odom",1);
  laserPublisher = n.advertise<sensor_msgs::LaserScan>("/laser", 1);
  mapLinesPublisher = n.advertise<visualization_msgs::Marker>(
      "/simulator_visualization", 6);
  posMarkerPublisher = n.advertise<visualization_msgs::Marker>(
      "/simulator_visualization", 6);
  dirMarkerPublisher = n.advertise<visualization_msgs::Marker>(
      "/simulator_visualization", 6);
  br = new tf::TransformBroadcaster();
}

void CobotSim::InitalLocationCallback(const PoseWithCovarianceStamped& msg) {
  curLoc.set(msg.pose.pose.position.x, msg.pose.pose.position.y);
  curAngle = 2.0 *
      atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
  printf("Set robot pose: %.2f,%.2f, %.1f\u00b0\n",
         curLoc.x,
         curLoc.y,
         DEG(curAngle));
}


/**
 * Helper method that initializes visualization_msgs::Marker parameters
 * @param vizMarker   pointer to the visualization_msgs::Marker object
 * @param ns          namespace for marker (string)
 * @param id          id of marker (int) - must be unique for each marker;
 *                      0, 1, and 2 are already used
 * @param type        specifies type of marker (string); available options:
 *                      arrow (default), cube, sphere, cylinder, linelist,
 *                      linestrip, points
 * @param p           stamped pose to define location and frame of marker
 * @param scale       scale of the marker; see visualization_msgs::Marker
 *                      documentation for details on the parameters
 * @param duration    lifetime of marker in RViz (double); use duration of 0.0
 *                      for infinite lifetime
 * @param color       vector of 4 float values representing color of marker;
 *                    0: red, 1: green, 2: blue, 3: alpha
 */
void CobotSim::initVizMarker(visualization_msgs::Marker& vizMarker, string ns,
    int id, string type, geometry_msgs::PoseStamped p,
    geometry_msgs::Point32 scale, double duration, vector<float> color) {

  vizMarker.header.frame_id = p.header.frame_id;
  vizMarker.header.stamp = ros::Time::now();

  vizMarker.ns = ns;
  vizMarker.id = id;

  if (type == "arrow") {
    vizMarker.type = visualization_msgs::Marker::ARROW;
  } else if (type == "cube") {
    vizMarker.type = visualization_msgs::Marker::CUBE;
  } else if (type == "sphere") {
    vizMarker.type = visualization_msgs::Marker::SPHERE;
  } else if (type == "cylinder") {
    vizMarker.type = visualization_msgs::Marker::CYLINDER;
  } else if (type == "linelist") {
    vizMarker.type = visualization_msgs::Marker::LINE_LIST;
  } else if (type == "linestrip") {
    vizMarker.type = visualization_msgs::Marker::LINE_STRIP;
  } else if (type == "points") {
    vizMarker.type = visualization_msgs::Marker::POINTS;
  } else {
    vizMarker.type = visualization_msgs::Marker::ARROW;
  }

  vizMarker.pose = p.pose;
  vizMarker.points.clear();
  vizMarker.scale.x = scale.x;
  vizMarker.scale.y = scale.y;
  vizMarker.scale.z = scale.z;

  vizMarker.lifetime = ros::Duration(duration);

  vizMarker.color.r = color.at(0);
  vizMarker.color.g = color.at(1);
  vizMarker.color.b = color.at(2);
  vizMarker.color.a = color.at(3);

  vizMarker.action = visualization_msgs::Marker::ADD;
}

void CobotSim::initCobotSimVizMarkers() {
  geometry_msgs::PoseStamped p;
  geometry_msgs::Point32 scale;
  vector<float> color;
  color.resize(4);

  p.header.frame_id = "/map";

  p.pose.orientation.w = 1.0;
  scale.x = 0.1;
  scale.y = 0.0;
  scale.z = 0.0;
  color[0] = 0.0;
  color[1] = 0.0;
  color[2] = 1.0;
  color[3] = 1.0;
  initVizMarker(lineListMarker, "map_lines", 0, "linelist", p, scale, 0.0, color);

  p.pose.position.z = CobotSim::robotHeight / 2.0;
  scale.x = cAxisLength;
  scale.y = cAxisLength / 2;
  scale.z = CobotSim::robotHeight;
  color[1] = 0.0;
  color[2] = 0.0;
  color[3] = 1.0;
  initVizMarker(robotPosMarker, "robot_position", 1, "cube", p, scale, 0.0, color);

  scale.x = cAxisLength + 0.4;
  scale.y = cAxisLength / 10;
  scale.z = 0.1;
  color[0] = 0.0;
  color[1] = 1.0;
  color[2] = 0.0;
  color[3] = 1.0;
  initVizMarker(robotDirMarker, "robot_direction", 2, "arrow", p, scale, 0.0, color);

}

void CobotSim::loadAtlas() {
  VectorMap currentMap = VectorMap(cMapName, "./maps", false);
  vector<line2f> map_segments = currentMap.Lines();
  for (size_t i = 0; i < map_segments.size(); i++) {
    // The line list needs two points for each line
    geometry_msgs::Point p0, p1;

    p0.x = map_segments.at(i).P0().x;
    p0.y = map_segments.at(i).P0().y;
    p0.z = 0.01;

    p1.x = map_segments.at(i).P1().x;
    p1.y = map_segments.at(i).P1().y;
    p1.z = 0.01;

    lineListMarker.points.push_back(p0);
    lineListMarker.points.push_back(p1);
  }
}

void CobotSim::AckermanDriveCallback(const AckermanDriveMsgConstPtr& msg) {
 if (!isfinite(msg->v) || !isfinite(msg->R)) {
    printf("Ignoring non-finite drive values: %f %f\n", msg->v, msg->R);
    return;
  }
  const double desired_speed = min(static_cast<double>(msg->v),
                                   transLimits.max_vel);

  const double inverse_min_radius = 1.0 / cMinTurnR;
  double desired_radius = 0;
  if (static_cast<double>(msg->R) > 0) {
    desired_radius = min(static_cast<double>(msg->R), inverse_min_radius);
  }

  double dv_max = 0.0;
  if (desired_speed > vel) {
    dv_max = cDT * transLimits.max_accel;
  }

  double dv = desired_speed - vel;
  if(fabs(dv) > dv_max) {
    dv = sign(dv) * dv_max;
  }
  vel = vel + dv;

  const double steering_angle = atan(desired_radius * cAxisLength);
  const double v_theta = (tan(steering_angle)/cAxisLength) * vel;
  const double v_x = cos(curAngle) * vel;
  const double v_y = sin(curAngle) * vel;

  curLoc.x += v_x * cDT;
  curLoc.y += v_y * cDT;
  curAngle = angle_mod(curAngle + v_theta * cDT);

  tLastCmd = GetTimeSec();
}

void CobotSim::publishOdometry() {
  static const double kMaxCommandAge = 0.5;

  if (GetTimeSec() > tLastCmd + kMaxCommandAge) {
    angVel = 0.0;
    vel = 0.0;
  }

  tf::Quaternion robotQ = tf::createQuaternionFromYaw(curAngle);

  odometryTwistMsg.header.stamp = ros::Time::now();
  odometryTwistMsg.pose.pose.position.x = curLoc.x;
  odometryTwistMsg.pose.pose.position.y = curLoc.y;
  odometryTwistMsg.pose.pose.position.z = 0.0;
  odometryTwistMsg.pose.pose.orientation.x = robotQ.x();
  odometryTwistMsg.pose.pose.orientation.y = robotQ.y();
  odometryTwistMsg.pose.pose.orientation.z = robotQ.z();;
  odometryTwistMsg.pose.pose.orientation.w = robotQ.w();
  odometryTwistMsg.twist.twist.angular.x = 0.0;
  odometryTwistMsg.twist.twist.angular.y = 0.0;
  odometryTwistMsg.twist.twist.angular.z = angVel;
  odometryTwistMsg.twist.twist.linear.x = vel * cos(curAngle);
  odometryTwistMsg.twist.twist.linear.y = vel * sin(curAngle);
  odometryTwistMsg.twist.twist.linear.z = 0.0;

  odometryTwistPublisher.publish(odometryTwistMsg);

  robotPosMarker.pose.position.x = curLoc.x + (cos(curAngle)*0.5*cAxisLength);
  robotPosMarker.pose.position.y = curLoc.y + (sin(curAngle)*0.5*cAxisLength);
  robotPosMarker.pose.position.z = CobotSim::robotHeight / 2.0;
  robotPosMarker.pose.orientation.w = 1.0;
  robotPosMarker.pose.orientation.x = robotQ.x();
  robotPosMarker.pose.orientation.y = robotQ.y();
  robotPosMarker.pose.orientation.z = robotQ.z();
  robotPosMarker.pose.orientation.w = robotQ.w();

  robotDirMarker.pose.position.x = curLoc.x;
  robotDirMarker.pose.position.y = curLoc.y;
  robotDirMarker.pose.position.z = CobotSim::robotHeight;
  robotDirMarker.pose.orientation.x = robotQ.x();
  robotDirMarker.pose.orientation.y = robotQ.y();
  robotDirMarker.pose.orientation.z = robotQ.z();
  robotDirMarker.pose.orientation.w = robotQ.w();

}

void CobotSim::publishLaser() {
  VectorMap currentMap = VectorMap(cMapName, "./maps", false);

  scanDataMsg.header.stamp = ros::Time::now();
  vector2f laserLoc(0.145,0.0);
  // ROS_INFO("curLoc: (%4.3f, %4.3f)", curLoc.x, curLoc.y);
  laserLoc = curLoc + laserLoc.rotate(curAngle);
  vector<float> ranges =
      currentMap.getRayCast(laserLoc,curAngle,RAD(360.0)/1024.0,769,0.02,4.0);
  scanDataMsg.ranges.resize(ranges.size());
  for(int i=0; i<int(ranges.size()); i++) {
    scanDataMsg.ranges[i] = ranges[i];
    if(ranges[i]>3.5) {
      scanDataMsg.ranges[i] = 0.0;
    }
  }
  laserPublisher.publish(scanDataMsg);
}

void CobotSim::publishTransform() {
  tf::Transform transform;
  tf::Quaternion q;

  transform.setOrigin(tf::Vector3(0.0,0.0,0.0));
  transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
  br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "/odom"));

  transform.setOrigin(tf::Vector3(curLoc.x,curLoc.y,0.0));
  q.setRPY(0.0,0.0,curAngle);
  transform.setRotation(q);
  br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/odom", "/base_footprint"));

  transform.setOrigin(tf::Vector3(0.0 ,0.0, 0.0));
  transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
  br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/base_footprint", "/base_link"));

  transform.setOrigin(tf::Vector3(0.145,0.0, 0.23));
  transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1));
  br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/base_link", "/base_laser"));
}

void CobotSim::publishVisualizationMarkers() {
  mapLinesPublisher.publish(lineListMarker);
  dirMarkerPublisher.publish(robotDirMarker);
  posMarkerPublisher.publish(robotPosMarker);
}

void CobotSim::run() {
  //publish odometry and status
  publishOdometry();
  //publish laser rangefinder messages
  publishLaser();
  // publish visualization marker messages
  publishVisualizationMarkers();
  //publish tf
  publishTransform();
}
