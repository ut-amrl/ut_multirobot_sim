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
\brief   C++ Implementation: Simulator
\author  Joydeep Biswas, (C) 2011
*/
//========================================================================

#include <math.h>
#include <memory>
#include <stdio.h>

#include <random>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "gflags/gflags.h"

#include "simulator.h"
#include "simulator/ackermann_model.h"
#include "simulator/cobot_model.h"
#include "simulator/diff_drive_model.h"
#include "config_reader/config_reader.h"
#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/ros/ros_helpers.h"
#include "shared/util/timer.h"
#include "vector_map.h"

DEFINE_bool(localize, false, "Publish localization");

using Eigen::Rotation2Df;
using Eigen::Vector2f;
using geometry::Heading;
using geometry::Line2f;
using geometry_msgs::PoseWithCovarianceStamped;
using math_util::AngleMod;
using math_util::DegToRad;
using math_util::RadToDeg;
using ackermann::AckermannModel;
using std::atan2;
using cobot::CobotModel;
using diffdrive::DiffDriveModel;
using vector_map::VectorMap;
using human::HumanObject;

CONFIG_STRING(map_name, "map_name");
// Used for visualizations
CONFIG_FLOAT(car_length, "car_length");
CONFIG_FLOAT(car_width, "car_width");
CONFIG_FLOAT(car_height, "car_height");
CONFIG_FLOAT(rear_axle_offset, "rear_axle_offset");
// Used for transforms
CONFIG_FLOAT(laser_x, "laser_loc.x");
CONFIG_FLOAT(laser_y, "laser_loc.y");
CONFIG_FLOAT(laser_z, "laser_loc.z");
// Initial location
CONFIG_FLOAT(start_x, "start_x");
CONFIG_FLOAT(start_y, "start_y");
CONFIG_FLOAT(start_angle, "start_angle");
// Timestep size
CONFIG_FLOAT(DT, "delta_t");
CONFIG_FLOAT(laser_stdev, "laser_noise_stddev");
// TF publications
CONFIG_BOOL(publish_map_to_odom, "publish_map_to_odom");
CONFIG_BOOL(publish_foot_to_base, "publish_foot_to_base");

// Used for topic names and robot specs
CONFIG_INT(robot_type, "robot_type");
CONFIG_STRING(laser_topic, "laser_topic");
const vector<string> config_list = {"config/sim_config.lua",
                                    "config/ackermann_config.lua",
                                    "config/cobot_config.lua",
                                    "config/bwibot_config.lua"};
config_reader::ConfigReader reader(config_list);

/* const vector<string> object_config_list = {"config/human_config.lua"};
config_reader::ConfigReader object_reader(object_config_list); */

Simulator::Simulator() :
    vel_(0, {0,0}),
    cur_loc_(0, {0,0}),
    laser_noise_(0, 1),
    robot_type_(static_cast<RobotType>(CONFIG_robot_type)) {
  truePoseMsg.header.seq = 0;
  truePoseMsg.header.frame_id = "map";
}

Simulator::~Simulator() { }

void Simulator::init(ros::NodeHandle& n) {
  // TODO(jaholtz) Too much hard coding, move to config
  scanDataMsg.header.seq = 0;
  scanDataMsg.header.frame_id = "base_laser";
  scanDataMsg.angle_min = DegToRad(-135.0);
  scanDataMsg.angle_max = DegToRad(135.0);
  scanDataMsg.range_min = 0.02;
  scanDataMsg.range_max = 10.0;
  scanDataMsg.angle_increment = DegToRad(0.25);
  scanDataMsg.intensities.clear();
  scanDataMsg.time_increment = 0.0;
  scanDataMsg.scan_time = 0.05;

  odometryTwistMsg.header.seq = 0;
  odometryTwistMsg.header.frame_id = "odom";
  odometryTwistMsg.child_frame_id = "base_footprint";

  cur_loc_ = Pose2Df(CONFIG_start_angle, {CONFIG_start_x, CONFIG_start_y});

  // Create motion model based on robot type
  // TODO extend to handle the multi-robot case
  switch(robot_type_) {
    case F1TEN:
      motion_model_ =
          unique_ptr<AckermannModel>(new AckermannModel(config_list, &n));
      break;
    case COBOT:
      motion_model_ =
          unique_ptr<CobotModel>(new CobotModel(config_list, &n));
      break;
    case BWIBOT:
      motion_model_ =
          unique_ptr<DiffDriveModel>(new DiffDriveModel(config_list, &n));
      break;
  }
  motion_model_->SetPose(cur_loc_);
  initSimulatorVizMarkers();
  drawMap();

  initSubscriber = n.subscribe(
      "/initialpose", 1, &Simulator::InitalLocationCallback, this);
  odometryTwistPublisher = n.advertise<nav_msgs::Odometry>("/odom",1);
  laserPublisher = n.advertise<sensor_msgs::LaserScan>(CONFIG_laser_topic, 1);
  viz_laser_publisher_ = n.advertise<sensor_msgs::LaserScan>("/scan", 1);
  mapLinesPublisher = n.advertise<visualization_msgs::Marker>(
      "/simulator_visualization", 6);
  posMarkerPublisher = n.advertise<visualization_msgs::Marker>(
      "/simulator_visualization", 6);
  objectLinesPublisher = n.advertise<visualization_msgs::Marker>(
      "/simulator_visualization", 6);
  truePosePublisher = n.advertise<geometry_msgs::PoseStamped>(
      "/simulator_true_pose", 1);
  localizationPublisher = n.advertise<geometry_msgs::Pose2D>(
      "/localization", 1);
  br = new tf::TransformBroadcaster();

  this->loadObject();
}

// TODO(yifeng): Change this into a general way
void Simulator::loadObject() {
  ShortTermObject* shortTermObject =
      new ShortTermObject("short_term_config.lua");
  objects.push_back(shortTermObject);

  const vector<string> object_config_0 = {"config/human_config.lua"};
  HumanObject* human_object0 = new HumanObject(object_config_0);
  objects.push_back(human_object0);
}

void Simulator::InitalLocationCallback(const PoseWithCovarianceStamped& msg) {
  const Vector2f loc =
      Vector2f(msg.pose.pose.position.x, msg.pose.pose.position.y);
  const float angle = 2.0 *
      atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
  printf("Set robot pose: %.2f,%.2f, %.1f\u00b0\n",
         cur_loc_.translation.x(),
         cur_loc_.translation.y(),
         RadToDeg(cur_loc_.angle));
  motion_model_->SetPose({angle, loc});
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
void Simulator::initVizMarker(visualization_msgs::Marker& vizMarker, string ns,
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

void Simulator::initSimulatorVizMarkers() {
  geometry_msgs::PoseStamped p;
  geometry_msgs::Point32 scale;
  vector<float> color;
  color.resize(4);

  p.header.frame_id = "/map";

  p.pose.orientation.w = 1.0;
  scale.x = 0.02;
  scale.y = 0.0;
  scale.z = 0.0;
  color[0] = 66.0 / 255.0;
  color[1] = 134.0 / 255.0;
  color[2] = 244.0 / 255.0;
  color[3] = 1.0;
  initVizMarker(lineListMarker, "map_lines", 0, "linelist", p, scale, 0.0,
      color);

  p.pose.position.z = 0.5 * CONFIG_car_height;
  scale.x = CONFIG_car_length;
  scale.y = CONFIG_car_width;
  scale.z = CONFIG_car_height;
  color[0] = 94.0 / 255.0;
  color[1] = 156.0 / 255.0;
  color[2] = 255.0 / 255.0;
  color[3] = 0.8;
  initVizMarker(robotPosMarker, "robot_position", 1, "cube", p, scale, 0.0,
      color);

  p.pose.orientation.w = 1.0;
  scale.x = 0.02;
  scale.y = 0.0;
  scale.z = 0.0;
  color[0] = 244.0 / 255.0;
  color[1] = 0.0 / 255.0;
  color[2] = 156.0 / 255.0;
  color[3] = 1.0;
  initVizMarker(objectLinesMarker, "object_lines", 0, "linelist", p, scale,
      0.0, color);
}

void Simulator::drawMap() {
  ros_helpers::ClearMarker(&lineListMarker);
  for (const Line2f& l : map_.lines) {
    ros_helpers::DrawEigen2DLine(l.p0, l.p1, &lineListMarker);
  }
}

void Simulator::drawObjects() {
  // draw objects
  ros_helpers::ClearMarker(&objectLinesMarker);
  for (const Line2f& l : map_.object_lines) {
    ros_helpers::DrawEigen2DLine(l.p0, l.p1, &objectLinesMarker);
  }
}

void Simulator::publishOdometry() {
  tf::Quaternion robotQ = tf::createQuaternionFromYaw(cur_loc_.angle);

  odometryTwistMsg.header.stamp = ros::Time::now();
  odometryTwistMsg.pose.pose.position.x = cur_loc_.translation.x();
  odometryTwistMsg.pose.pose.position.y = cur_loc_.translation.y();
  odometryTwistMsg.pose.pose.position.z = 0.0;
  odometryTwistMsg.pose.pose.orientation.x = robotQ.x();
  odometryTwistMsg.pose.pose.orientation.y = robotQ.y();
  odometryTwistMsg.pose.pose.orientation.z = robotQ.z();
  odometryTwistMsg.pose.pose.orientation.w = robotQ.w();
  odometryTwistMsg.twist.twist.angular.x = 0.0;
  odometryTwistMsg.twist.twist.angular.y = 0.0;
  odometryTwistMsg.twist.twist.angular.z = vel_.angle;
  odometryTwistMsg.twist.twist.linear.x = vel_.translation.x();
  odometryTwistMsg.twist.twist.linear.y = vel_.translation.y();
  odometryTwistMsg.twist.twist.linear.z = 0.0;

  odometryTwistPublisher.publish(odometryTwistMsg);

  // TODO(jaholtz) visualization should not always be based on car
  // parameters
  robotPosMarker.pose.position.x =
      cur_loc_.translation.x() - cos(cur_loc_.angle) * CONFIG_rear_axle_offset;
  robotPosMarker.pose.position.y =
      cur_loc_.translation.y() - sin(cur_loc_.angle) * CONFIG_rear_axle_offset;
  robotPosMarker.pose.position.z = 0.5 * CONFIG_car_height;
  robotPosMarker.pose.orientation.w = 1.0;
  robotPosMarker.pose.orientation.x = robotQ.x();
  robotPosMarker.pose.orientation.y = robotQ.y();
  robotPosMarker.pose.orientation.z = robotQ.z();
  robotPosMarker.pose.orientation.w = robotQ.w();
}

void Simulator::publishLaser() {
  if (map_.file_name != CONFIG_map_name) {
    map_.Load(CONFIG_map_name);
    drawMap();
  }
  scanDataMsg.header.stamp = ros::Time::now();
  const Vector2f laserRobotLoc(CONFIG_laser_x, CONFIG_laser_y);
  const Vector2f laserLoc =
      cur_loc_.translation + Rotation2Df(cur_loc_.angle) * laserRobotLoc;

  const int num_rays = static_cast<int>(
      1.0 + (scanDataMsg.angle_max - scanDataMsg.angle_min) /
      scanDataMsg.angle_increment);
  map_.GetPredictedScan(laserLoc,
                        scanDataMsg.range_min,
                        scanDataMsg.range_max,
                        scanDataMsg.angle_min + cur_loc_.angle,
                        scanDataMsg.angle_max + cur_loc_.angle,
                        num_rays,
                        &scanDataMsg.ranges);
  for (float& r : scanDataMsg.ranges) {
    if (r > scanDataMsg.range_max - 0.1) {
      r = scanDataMsg.range_max;
      continue;
    }
    r = max<float>(0.0, r + CONFIG_laser_stdev * laser_noise_(rng_));
  }

  // TODO Avoid publishing laser twice.
  // Currently publishes once for the visualizer and once for robot
  // requirements.
  laserPublisher.publish(scanDataMsg);
  viz_laser_publisher_.publish(scanDataMsg);
}

void Simulator::publishTransform() {
  tf::Transform transform;
  tf::Quaternion q;
  
  if(CONFIG_publish_map_to_odom) {
      transform.setOrigin(tf::Vector3(0.0,0.0,0.0));
      transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
      br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map",
      "/odom"));
  }
  transform.setOrigin(tf::Vector3(cur_loc_.translation.x(),
        cur_loc_.translation.y(), 0.0));
  q.setRPY(0.0, 0.0, cur_loc_.angle);
  transform.setRotation(q);
  br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/odom",
      "/base_footprint"));
  
  if(CONFIG_publish_foot_to_base){
      transform.setOrigin(tf::Vector3(0.0 ,0.0, 0.0));
      transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
      br->sendTransform(tf::StampedTransform(transform, ros::Time::now(),
        "/base_footprint", "/base_link"));
  }
  
  transform.setOrigin(tf::Vector3(CONFIG_laser_x,
        CONFIG_laser_y, CONFIG_laser_z));
  transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1));
  br->sendTransform(tf::StampedTransform(transform, ros::Time::now(),
      "/base_link", "/base_laser"));
}

void Simulator::publishVisualizationMarkers() {
  mapLinesPublisher.publish(lineListMarker);
  posMarkerPublisher.publish(robotPosMarker);
  objectLinesPublisher.publish(objectLinesMarker);
}

void Simulator::update() {
  // Step the motion model forward one time step
  motion_model_->Step(CONFIG_DT);

  // Update the simulator with the motion model result.
  cur_loc_ = motion_model_->GetPose();
  vel_ = motion_model_->GetVel();

  // Publishing the ground truth pose
  truePoseMsg.header.stamp = ros::Time::now();
  truePoseMsg.pose.position.x = cur_loc_.translation.x();
  truePoseMsg.pose.position.y = cur_loc_.translation.y();
  truePoseMsg.pose.position.z = 0;
  truePoseMsg.pose.orientation.w = cos(0.5 * cur_loc_.angle);
  truePoseMsg.pose.orientation.z = sin(0.5 * cur_loc_.angle);
  truePoseMsg.pose.orientation.x = 0;
  truePoseMsg.pose.orientation.y = 0;
  truePosePublisher.publish(truePoseMsg);

  // Update all map objects and get their lines
  map_.object_lines.clear();
  for (size_t i=0; i < objects.size(); i++){
    objects[i]->Step(CONFIG_DT);
    auto pose_lines = objects[i]->GetLines();
    for (Line2f line: pose_lines){
      map_.object_lines.push_back(line);
    }
  }
  this->drawObjects();
}

void Simulator::Run() {
  // Simulate time-step.
  update();
  //publish odometry and status
  publishOdometry();
  //publish laser rangefinder messages
  publishLaser();
  // publish visualization marker messages
  publishVisualizationMarkers();
  //publish tf
  publishTransform();

  if (FLAGS_localize) {
    geometry_msgs::Pose2D localization_msg;
    localization_msg.x = cur_loc_.translation.x();
    localization_msg.y = cur_loc_.translation.y();
    localization_msg.theta = cur_loc_.angle;
    localizationPublisher.publish(localization_msg);
  }
}
