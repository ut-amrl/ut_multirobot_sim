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

#include <libgen.h>
#include <math.h>
#include <memory>
#include <stdio.h>

#include <iterator>
#include <random>
#include <string>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "gflags/gflags.h"

#include "simulator.h"
#include "simulator/ackermann_model.h"
#include "simulator/cobot_model.h"
#include "simulator/diff_drive_model.h"
#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/ros/ros_helpers.h"
#include "shared/util/timer.h"
#include "ut_multirobot_sim/Localization2DMsg.h"
#include "vector_map.h"

DEFINE_bool(localize, false, "Publish localization");

using ackermann::AckermannModel;
using cobot::CobotModel;
using Eigen::Rotation2Df;
using Eigen::Vector2f;
using Eigen::Vector3f;
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

CONFIG_STRING(init_config_file, "init_config_file");
// Used for visualizations
CONFIG_FLOAT(car_length, "car_length");
CONFIG_FLOAT(car_width, "car_width");
CONFIG_FLOAT(car_height, "car_height");
CONFIG_FLOAT(rear_axle_offset, "rear_axle_offset");
// Used for transforms
CONFIG_FLOAT(laser_x, "laser_loc.x");
CONFIG_FLOAT(laser_y, "laser_loc.y");
CONFIG_FLOAT(laser_z, "laser_loc.z");
// Timestep size
CONFIG_FLOAT(DT, "delta_t");
CONFIG_FLOAT(laser_stdev, "laser_noise_stddev");
// TF publications
CONFIG_BOOL(publish_tfs, "publish_tfs");
CONFIG_BOOL(publish_map_to_odom, "publish_map_to_odom");
CONFIG_BOOL(publish_foot_to_base, "publish_foot_to_base");

// Used for topic names and robot specs
CONFIG_INTLIST(robot_types, "type_list");
CONFIG_STRING(robot_configs, "config_list");
CONFIG_STRING(laser_topic, "laser_topic");
CONFIG_STRING(laser_frame, "laser_frame");
CONFIG_VECTOR3FLIST(robot_colors, "RGB_list");
CONFIG_INT(robot_number, "robot_number");
CONFIG_STRINGLIST(topic_prefixs, "topic_prefix_list");

CONFIG_STRING(map_name, "map_name");
// Initial location and angles
CONFIG_VECTOR2FLIST(robot_start_locations, "location_list");
CONFIG_DOUBLELIST(robot_start_angles, "angle_list");

CONFIG_STRINGLIST(short_term_object_config_list, "short_term_object_config_list");
CONFIG_STRINGLIST(human_config_list, "human_config_list");

/* const vector<string> object_config_list = {"config/human_config.lua"};
config_reader::ConfigReader object_reader(object_config_list); */

Simulator::Simulator(const std::string& sim_config) :
    reader_({sim_config}),
    init_config_reader_({CONFIG_init_config_file}),
    vel_(0, {0,0}),
    cur_loc_(0, {0,0}),
    laser_noise_(0, 1),
    robot_number_(CONFIG_robot_number),
    topic_prefixs_(CONFIG_topic_prefixs),
    robot_types_(CONFIG_robot_types){
  truePoseMsg.header.seq = 0;
  truePoseMsg.header.frame_id = "map";
  // TODO: add static cast robot_type_(static_cast<RobotType>(CONFIG_robot_type))
  // initialize object lines associated with each robot
  motion_models_lines_.resize(robot_number_, std::vector<Line2f>(0));
}

Simulator::~Simulator() {}

void Simulator::init(ros::NodeHandle& n) {
  // TODO(jaholtz) Too much hard coding, move to config
  scanDataMsg.header.seq = 0;
  scanDataMsg.header.frame_id = CONFIG_laser_frame;
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

  for (int i = 0; i < robot_number_; i++) {
    string robot_type = robot_types_[i];
    string topic_prefix = topic_prefixs_[i];
    if (robot_type == "F1TEN") {
      motion_models_.push_back(unique_ptr<AckermannModel>(
          new AckermannModel({CONFIG_robot_configs[i]}, &n, topic_prefix)));
    } else if (robot_type == "COBOT"){
      motion_models_.push_back(unique_ptr<CobotModel>(
          new CobotModel({CONFIG_robot_configs[i]}, &n, topic_prefix)));
    }
    else if (robot_type_ == "BWIBOT") {
      motion_models_.push_back(unique_ptr<DiffDriveModel>(
          new DiffDriveModel({CONFIG_robot_configs[i]}, &n, topic_prefix)));
    }
    // initialize starting location
    cur_locs_.push_back(
        Pose2Df(CONFIG_robot_start_angles[i], CONFIG_robot_start_locations[i]));
    motion_models_[i]->SetPose(cur_locs_[i]);
  }

  initSimulatorVizMarkers(); // TODO: need to add a vis marker for every robot
  drawMap();

  // Initialize topics for each robot
  for(int i = 0; i < robot_number_; i++){
    string topic_prefix = topic_prefixs_[i];
    printf("[%d] topic_prefix: %s\n", i, topic_prefix.c_str());
    // TODO: Change this hardcoding to something nicer
        odometryTwistPublishers.push_back(
        n.advertise<nav_msgs::Odometry>(topic_prefix + "/odom", 1));
    laserPublishers.push_back(n.advertise<sensor_msgs::LaserScan>(
        topic_prefix + CONFIG_laser_topic, 1));
    viz_laser_publishers_.push_back(n.advertise<sensor_msgs::LaserScan>(
        topic_prefix + "/scan", 1));
    if (FLAGS_localize) {
      localizationPublishers.push_back(n.advertise<ut_multirobot_sim::Localization2DMsg>(
          topic_prefix + "/localization", 1));
    }
  }
  // initialize hard coded initial pose
  for (int i = 0; i < robot_number_; i++) {
    if (i == 7) {
      initSubscribers.push_back(
          n.subscribe(topic_prefixs_[7] + "/initialpose", 1,
                      &Simulator::InitalLocationCallbackCar7, this));
    } else if (i == 6) {
      initSubscribers.push_back(
          n.subscribe(topic_prefixs_[6] + "/initialpose", 1,
                      &Simulator::InitalLocationCallbackCar6, this));
    } else if (i == 5) {
      initSubscribers.push_back(
          n.subscribe(topic_prefixs_[5] + "/initialpose", 1,
                      &Simulator::InitalLocationCallbackCar5, this));
    } else if (i == 4) {
      initSubscribers.push_back(
          n.subscribe(topic_prefixs_[4] + "/initialpose", 1,
                      &Simulator::InitalLocationCallbackCar4, this));
    } else if (i == 3) {
      initSubscribers.push_back(
          n.subscribe(topic_prefixs_[3] + "/initialpose", 1,
                      &Simulator::InitalLocationCallbackCar3, this));
    } else if (i == 2) {
      initSubscribers.push_back(
          n.subscribe(topic_prefixs_[2] + "/initialpose", 1,
                      &Simulator::InitalLocationCallbackCar2, this));
    } else if (i == 1) {
      initSubscribers.push_back(
          n.subscribe(topic_prefixs_[1] + "/initialpose", 1,
                      &Simulator::InitalLocationCallbackCar1, this));
    } else if (i == 0) {
      initSubscribers.push_back(
          n.subscribe(topic_prefixs_[0] + "/initialpose", 1,
                      &Simulator::InitalLocationCallbackCar0, this));
    }
  }
  // initialize topics shared jointly
  truePosePublisher = n.advertise<geometry_msgs::PoseStamped>(
        "/simulator_true_pose", 1); 

  posMarkerPublisher = n.advertise<visualization_msgs::Marker>(
      "/simulator_visualization", 6); 
  mapLinesPublisher = n.advertise<visualization_msgs::Marker>(
      "/simulator_visualization", 6);
  objectLinesPublisher = n.advertise<visualization_msgs::Marker>(
      "/simulator_visualization", 6);
  if (FLAGS_localize) {
    localizationMsg.header.frame_id = "map";
    localizationMsg.header.seq = 0;
  }


  br = new tf::TransformBroadcaster();

  this->loadObject();
}


// TODO(yifeng): Change this into a general way
void Simulator::loadObject() {
  // TODO (yifeng): load short term objects from list
  objects.push_back(
    std::unique_ptr<ShortTermObject>(new ShortTermObject("short_term_config.lua")));

  // human
  for (const string& config_str: CONFIG_human_config_list) {
    objects.push_back(
      std::unique_ptr<HumanObject>(new HumanObject({config_str})));

  }
}

// TODO: figure out higher order function
void Simulator::InitalLocationCallbackMulti(const PoseWithCovarianceStamped& msg, int car_num) {
  const Vector2f loc =
      Vector2f(msg.pose.pose.position.x, msg.pose.pose.position.y);
  const float angle = 2.0 *
      atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
  printf("Set robot pose: %.2f,%.2f, %.1f\u00b0\n",
         cur_loc_.translation.x(),
         cur_loc_.translation.y(),
         RadToDeg(cur_loc_.angle));
  motion_models_[car_num]->SetPose({angle, loc}); // TODO, add loop
}

void Simulator::InitalLocationCallbackCar0(
    const PoseWithCovarianceStamped &msg) {
  InitalLocationCallbackMulti(msg, 0);
}

void Simulator::InitalLocationCallbackCar1(
    const PoseWithCovarianceStamped &msg) {
  InitalLocationCallbackMulti(msg, 1);
}

void Simulator::InitalLocationCallbackCar2(
    const PoseWithCovarianceStamped &msg) {
  InitalLocationCallbackMulti(msg, 2);
}

void Simulator::InitalLocationCallbackCar3(
    const PoseWithCovarianceStamped &msg) {
  InitalLocationCallbackMulti(msg, 3);
}

void Simulator::InitalLocationCallbackCar4(
    const PoseWithCovarianceStamped &msg) {
  InitalLocationCallbackMulti(msg, 4);
}

void Simulator::InitalLocationCallbackCar5(
    const PoseWithCovarianceStamped &msg) {
  InitalLocationCallbackMulti(msg, 5);
}

void Simulator::InitalLocationCallbackCar6(
    const PoseWithCovarianceStamped &msg) {
  InitalLocationCallbackMulti(msg, 6);
}

void Simulator::InitalLocationCallbackCar7(
    const PoseWithCovarianceStamped &msg) {
  InitalLocationCallbackMulti(msg, 7);
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
  color[0] = 0;
  color[1] = 0;
  color[2] = 0;
  color[3] = 0.8; // luminosity

  for(int i = 0; i < robot_number_; i++){
    robotPosMarkers.push_back(visualization_msgs::Marker());
    color[0] = CONFIG_robot_colors[i].x()/255.0;
    color[1] = CONFIG_robot_colors[i].y()/255.0;
    color[2] = CONFIG_robot_colors[i].z()/255.0;

    initVizMarker(robotPosMarkers[i],
                  "car" + std::to_string(i) + "_robot_position", i + 2, "cube",
                  p, scale, 0.0, color);
  }

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

void Simulator::publishOdometry(int cur_car_number) {
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
 
  odometryTwistMsg.header.frame_id = topic_prefixs_[cur_car_number] + "/odom";
  odometryTwistMsg.child_frame_id = topic_prefixs_[cur_car_number] + "/base_footprint";

  odometryTwistPublishers[cur_car_number].publish(odometryTwistMsg);

  // TODO(jaholtz) visualization should not always be based on car
  // parameters
  robotPosMarkers[cur_car_number].pose.position.x =
      cur_loc_.translation.x() - cos(cur_loc_.angle) * CONFIG_rear_axle_offset;
  robotPosMarkers[cur_car_number].pose.position.y =
      cur_loc_.translation.y() - sin(cur_loc_.angle) * CONFIG_rear_axle_offset;
  robotPosMarkers[cur_car_number].pose.position.z = 0.5 * CONFIG_car_height;
  robotPosMarkers[cur_car_number].pose.orientation.w = 1.0;
  robotPosMarkers[cur_car_number].pose.orientation.x = robotQ.x();
  robotPosMarkers[cur_car_number].pose.orientation.y = robotQ.y();
  robotPosMarkers[cur_car_number].pose.orientation.z = robotQ.z();
  robotPosMarkers[cur_car_number].pose.orientation.w = robotQ.w();
}

void Simulator::publishLaser(int cur_car_number) {
  if (map_.file_name != CONFIG_map_name) {
    map_.Load(CONFIG_map_name);
    drawMap();
  }
  scanDataMsg.header.stamp = ros::Time::now();
  scanDataMsg.header.frame_id = topic_prefixs_[cur_car_number] + "/base_laser";

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
  laserPublishers[cur_car_number].publish(scanDataMsg);
  viz_laser_publishers_[cur_car_number].publish(scanDataMsg);
}

void Simulator::publishTransform(int cur_car_number) {
  if (!CONFIG_publish_tfs) {
    return;
  }

  tf::Transform transform;
  tf::Quaternion q;

  string topic_prefix = topic_prefixs_[cur_car_number];

  if(CONFIG_publish_map_to_odom) {
    transform.setOrigin(tf::Vector3(0.0,0.0,0.0));
    transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
    br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map",
        topic_prefix + "/odom"));
  }

  transform.setOrigin(tf::Vector3(cur_loc_.translation.x(),
        cur_loc_.translation.y(), 0.0));
  q.setRPY(0.0, 0.0, cur_loc_.angle);
  transform.setRotation(q);
  br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), topic_prefix + "/odom",
      topic_prefix + "/base_footprint"));

  if(CONFIG_publish_foot_to_base){
    transform.setOrigin(tf::Vector3(0.0 ,0.0, 0.0));
    transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
    br->sendTransform(tf::StampedTransform(transform, ros::Time::now(),
        topic_prefix + "/base_footprint", topic_prefix + "/base_link"));
  }

  transform.setOrigin(tf::Vector3(CONFIG_laser_x,
        CONFIG_laser_y, CONFIG_laser_z));
  transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1));
  br->sendTransform(tf::StampedTransform(transform, ros::Time::now(),
      topic_prefix + "/base_link", topic_prefix + "/base_laser"));
}

void Simulator::publishVisualizationMarkers(int cur_car_number) {
  mapLinesPublisher.publish(lineListMarker);

  posMarkerPublisher.publish(robotPosMarkers[cur_car_number]);

  objectLinesPublisher.publish(objectLinesMarker);
}

void Simulator::updateLocation(int cur_car_number){
  // Step the motion model forward one time step
  motion_models_[cur_car_number]->Step(CONFIG_DT);
  // Update the simulator with the motion model result.
  cur_locs_[cur_car_number] = motion_models_[cur_car_number]->GetPose();
}

void Simulator::update(int cur_car_number) {
  cur_loc_ = cur_locs_[cur_car_number];
  vel_ = motion_models_[cur_car_number]->GetVel();

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
  // update objects associated with each robot
  for (Line2f line: motion_models_lines_[cur_car_number]){
      map_.object_lines.push_back(line);    
  }
}

void Simulator::updateSimulatorLines(){
  // Runtime is O(n^2),  but n (number of robot)
  // is always small, so this is fine.
  for(int i = 0; i < robot_number_; i++){
    motion_models_lines_[i].clear();
  }
  for(int i = 0; i < robot_number_; i++){
    for(int j = 0; j < robot_number_; j++){
      if(i != j){
        for(Line2f line: motion_models_[j]->GetLines()){
          motion_models_lines_[i].push_back(line);
        }
      }
    }
  }
}

string GetMapNameFromFilename(string path) {
  char path_cstring[path.length()];
  strcpy(path_cstring, path.c_str());
  const string file_name(basename(path_cstring));
  if (file_name.length() > 4 && 
      file_name.substr(file_name.length() - 4, 4) == ".txt") {
    return file_name.substr(0, file_name.length() - 4);
  }
  return file_name;
}



void Simulator::Run() {
  // update the position of the robots
  for(int i = 0; i < robot_number_; i++){
    updateLocation(i);
  }
  // update object lines associated with each robot
  this->updateSimulatorLines();
  // publish messages related to the robots
  for(int i = 0; i < robot_number_; i++){
    update(i);
    //publish odometry and status
    publishOdometry(i); 
    //publish laser rangefinder messages
    publishLaser(i);
    // publish visualization marker messages
    publishVisualizationMarkers(i);
    //publish tf
    publishTransform(i);
    if (FLAGS_localize) {
      localizationMsg.header.stamp = ros::Time::now();
      localizationMsg.map = GetMapNameFromFilename(map_.file_name);
      localizationMsg.pose.x = cur_loc_.translation.x();
      localizationMsg.pose.y = cur_loc_.translation.y();
      localizationMsg.pose.theta = cur_loc_.angle;
      localizationPublishers[i].publish(localizationMsg);
    }
  }
  
}
