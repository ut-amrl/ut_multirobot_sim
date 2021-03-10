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

#include <random>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "gflags/gflags.h"

#include "simulator.h"
#include "simulator/ackermann_model.h"
#include "simulator/door.h"
#include "simulator/omnidirectional_model.h"
#include "simulator/diff_drive_model.h"
#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/ros/ros_helpers.h"
#include "shared/util/timer.h"
#include "ut_multirobot_sim/HumanStateArrayMsg.h"
#include "ut_multirobot_sim/DoorArrayMsg.h"
#include "ut_multirobot_sim/DoorStateMsg.h"
#include "ut_multirobot_sim/Localization2DMsg.h"
#include "ut_multirobot_sim/DoorControlMsg.h"
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
using omnidrive::OmnidirectionalModel;
using diffdrive::DiffDriveModel;
using vector_map::VectorMap;
using human::HumanObject;
using door::Door;
using door::DoorState;
using ut_multirobot_sim::DoorControlMsg;

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
CONFIG_STRINGLIST(robot_types, "robot_types");
CONFIG_STRING(robot_config, "robot_config");
CONFIG_STRING(laser_topic, "laser_topic");
CONFIG_STRING(laser_frame, "laser_frame");

// Laser scanner parameters.
CONFIG_FLOAT(laser_angle_min, "laser_angle_min");
CONFIG_FLOAT(laser_angle_max, "laser_angle_max");
CONFIG_FLOAT(laser_angle_increment, "laser_angle_increment");
CONFIG_FLOAT(laser_min_range, "laser_min_range");
CONFIG_FLOAT(laser_max_range, "laser_max_range");

CONFIG_STRING(map_name, "map_name");
// Initial location
CONFIG_VECTOR3FLIST(start_poses, "start_poses");
CONFIG_STRINGLIST(short_term_object_config_list, "short_term_object_config_list");
CONFIG_STRINGLIST(human_config_list, "human_config_list");
CONFIG_STRINGLIST(door_config_list, "door_config_list");

/* const vector<string> object_config_list = {"config/human_config.lua"};
config_reader::ConfigReader object_reader(object_config_list); */

Simulator::Simulator(const std::string& sim_config) :
    reader_({sim_config}),
    init_config_reader_({CONFIG_init_config_file}),
    laser_noise_(0, 1),
    sim_step_count(0),
    sim_time(0.0) {
  truePoseMsg.header.seq = 0;
  truePoseMsg.header.frame_id = "map";
  if (CONFIG_map_name == "") {
    std::cerr << "Failed to load map from init config file '"
              << CONFIG_init_config_file << "'" << std::endl;
    exit(1);
  }
}

Simulator::~Simulator() { }

double Simulator::GetStepSize() const {
  return CONFIG_DT;
}

robot_model::RobotModel* MakeMotionModel(const std::string& robot_type,
                                         ros::NodeHandle& n,
                                         const std::string& topic_prefix) {
  if (robot_type == "ACKERMANN_DRIVE") {
      return new AckermannModel({CONFIG_robot_config}, &n);
    } else if (robot_type == "OMNIDIRECTIONAL_DRIVE") {
      return new OmnidirectionalModel({CONFIG_robot_config}, &n);
    } else if (robot_type == "DIFF_DRIVE") {
      return new DiffDriveModel({CONFIG_robot_config}, &n, topic_prefix);
    }
    std::cerr << "Robot type \"" << robot_type
              << "\" has no associated motion model!" << std::endl;
    return nullptr;
}

std::string IndexToPrefix(const size_t index) {
  return "robot" + std::to_string(index);
}

bool Simulator::init(ros::NodeHandle& n) {
  // TODO(jaholtz) Too much hard coding, move to config
  scanDataMsg.header.seq = 0;
  scanDataMsg.header.frame_id = CONFIG_laser_frame;
  scanDataMsg.angle_min = CONFIG_laser_angle_min;
  scanDataMsg.angle_max = CONFIG_laser_angle_max;
  scanDataMsg.angle_increment = CONFIG_laser_angle_increment;
  scanDataMsg.range_min = CONFIG_laser_min_range;
  scanDataMsg.range_max = CONFIG_laser_max_range;
  scanDataMsg.intensities.clear();
  scanDataMsg.time_increment = 0.0;
  scanDataMsg.scan_time = 0.05;

  odometryTwistMsg.header.seq = 0;
  odometryTwistMsg.header.frame_id = "odom";
  odometryTwistMsg.child_frame_id = "base_footprint";

  if (CONFIG_robot_types.size() != CONFIG_start_poses.size()) {
    std::cerr << "Robot type and robot start pose lists are"
                 "not the same size!" << std::endl;
    return false;
  }


  initSimulatorVizMarkers();
  drawMap();

  // Create motion model based on robot type
  for (size_t i = 0; i < CONFIG_start_poses.size(); ++i) {
    const auto& robot_type = CONFIG_robot_types.at(i);
    const auto& start_pose = CONFIG_start_poses.at(i);
    const auto pf = IndexToPrefix(i);
    auto* mm = MakeMotionModel(robot_type, n, pf);
    if (mm == nullptr) {
      return false;
    }
    mm->SetPose(Pose2Df(start_pose.z(), {start_pose.x(), start_pose.y()}));

    robot_pub_subs_.emplace_back(RobotPubSub());
    auto& rps = robot_pub_subs_.back();
    rps.motion_model = std::unique_ptr<robot_model::RobotModel>(mm);

    rps.initSubscriber = n.subscribe<ut_multirobot_sim::Localization2DMsg>(
       pf + "/initialpose", 1, [&](const boost::shared_ptr<const ut_multirobot_sim::Localization2DMsg>& msg) {
        const Vector2f loc(msg->pose.x, msg->pose.y);
        const float angle = msg->pose.theta;
        rps.motion_model->SetPose({angle, loc});
      });
    rps.odometryTwistPublisher = n.advertise<nav_msgs::Odometry>(pf + "/odom", 1);
    rps.laserPublisher = n.advertise<sensor_msgs::LaserScan>(pf + CONFIG_laser_topic, 1);
    rps.vizLaserPublisher = n.advertise<sensor_msgs::LaserScan>(pf + "/scan", 1);
    rps.posMarkerPublisher = n.advertise<visualization_msgs::Marker>(
        pf + "/simulator_visualization", 6);
    rps.truePosePublisher = n.advertise<geometry_msgs::PoseStamped>(
        pf + "/simulator_true_pose", 1);

      if (FLAGS_localize) {
        rps.localizationPublisher = n.advertise<ut_multirobot_sim::Localization2DMsg>(
            pf + "/localization", 1);
        localizationMsg.header.frame_id = "map";
        localizationMsg.header.seq = 0;
      }
  }

  mapLinesPublisher = n.advertise<visualization_msgs::Marker>("/simulator_visualization", 6);
  objectLinesPublisher = n.advertise<visualization_msgs::Marker>("/simulator_visualization", 6);


  doorSubscriber = n.subscribe("/door/command", 1, &Simulator::DoorCallback, this);

  // initSubscriber = n.subscribe(
      // "/initialpose", 1, &Simulator::InitalLocationCallback, this);
  // odometryTwistPublisher = n.advertise<nav_msgs::Odometry>("/odom",1);
  // laserPublisher = n.advertise<sensor_msgs::LaserScan>(CONFIG_laser_topic, 1);
  // viz_laser_publisher_ = n.advertise<sensor_msgs::LaserScan>("/scan", 1);
  // mapLinesPublisher = n.advertise<visualization_msgs::Marker>(
      // "/simulator_visualization", 6);
  // posMarkerPublisher = n.advertise<visualization_msgs::Marker>(
      // "/simulator_visualization", 6);
  // objectLinesPublisher = n.advertise<visualization_msgs::Marker>(
      // "/simulator_visualization", 6);
  // truePosePublisher = n.advertise<geometry_msgs::PoseStamped>(
      // "/simulator_true_pose", 1);
  // if (FLAGS_localize) {
    // localizationPublisher = n.advertise<ut_multirobot_sim::Localization2DMsg>(
        // "/localization", 1);
    // localizationMsg.header.frame_id = "map";
    // localizationMsg.header.seq = 0;
  // }
  humanStateArrayPublisher =
    n.advertise<ut_multirobot_sim::HumanStateArrayMsg>("/human_states", 1);
  doorStatePublisher =
    n.advertise<ut_multirobot_sim::DoorArrayMsg>("/door_states", 1);
  br = new tf::TransformBroadcaster();

  this->loadObject(n);
  return true;
}

// TODO(yifeng): Change this into a general way
void Simulator::loadObject(ros::NodeHandle& nh) {
  // TODO (yifeng): load short term objects from list
  objects.push_back(
    std::unique_ptr<ShortTermObject>(new ShortTermObject("short_term_config.lua")));

  // human
  for (const string& config_str: CONFIG_human_config_list) {
    objects.push_back(
      std::unique_ptr<HumanObject>(new HumanObject({config_str})));
  }

  // door
  for (const string& config_str : CONFIG_door_config_list) {
    objects.push_back(
      std::unique_ptr<Door>(new Door({config_str}))
    );
  }

  for (const std::unique_ptr<EntityBase>& e : objects) {
    if (e->GetType() == HUMAN_OBJECT) {
      EntityBase* e_raw = e.get();
      HumanObject* human = static_cast<HumanObject*>(e_raw);
      if (human->GetMode() == human::HumanMode::Controlled) {
        human->InitializeManualControl(nh);
      }
    }
  }
}

void Simulator::DoorCallback(const DoorControlMsg& msg) {
  DoorState state = static_cast<DoorState>(msg.command);
  for (const std::unique_ptr<EntityBase>& e : objects) {
    if (e->GetType() == DOOR) {
      EntityBase* e_raw = e.get();
      Door* door = static_cast<Door*>(e_raw);
      door->SetState(state);
    }
  }
}

void Simulator::InitalLocationCallback(const PoseWithCovarianceStamped& msg) {
  const Vector2f loc =
      Vector2f(msg.pose.pose.position.x, msg.pose.pose.position.y);
  const float angle = 2.0 *
      atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
  motion_model_->SetPose({angle, loc});
  printf("Set robot pose: %.2f,%.2f, %.1f\u00b0\n",
         loc.x(),
         loc.y(),
         RadToDeg(angle));
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

  p.header.frame_id = "map";

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

  for (auto& rps : robot_pub_subs_) {
    p.pose.position.z = 0.5 * CONFIG_car_height;
    scale.x = CONFIG_car_length;
    scale.y = CONFIG_car_width;
    scale.z = CONFIG_car_height;
    color[0] = 94.0 / 255.0;
    color[1] = 156.0 / 255.0;
    color[2] = 255.0 / 255.0;
    color[3] = 0.8;
    initVizMarker(rps.robotPosMarker, "robot_position", 1, "cube", p, scale, 0.0,
        color);
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

void Simulator::publishOdometry() {
  for (auto& rps : robot_pub_subs_) {
    tf::Quaternion robotQ = tf::createQuaternionFromYaw(rps.cur_loc.angle);

    odometryTwistMsg.header.stamp = ros::Time::now();
    odometryTwistMsg.pose.pose.position.x = rps.cur_loc.translation.x();
    odometryTwistMsg.pose.pose.position.y = rps.cur_loc.translation.y();
    odometryTwistMsg.pose.pose.position.z = 0.0;
    odometryTwistMsg.pose.pose.orientation.x = robotQ.x();
    odometryTwistMsg.pose.pose.orientation.y = robotQ.y();
    odometryTwistMsg.pose.pose.orientation.z = robotQ.z();
    odometryTwistMsg.pose.pose.orientation.w = robotQ.w();
    odometryTwistMsg.twist.twist.angular.x = 0.0;
    odometryTwistMsg.twist.twist.angular.y = 0.0;
    odometryTwistMsg.twist.twist.angular.z = rps.vel.angle;
    odometryTwistMsg.twist.twist.linear.x = rps.vel.translation.x();
    odometryTwistMsg.twist.twist.linear.y = rps.vel.translation.y();
    odometryTwistMsg.twist.twist.linear.z = 0.0;

    rps.odometryTwistPublisher.publish(odometryTwistMsg);

    // TODO(jaholtz) visualization should not always be based on car
    // parameters
    rps.robotPosMarker.pose.position.x =
        rps.cur_loc.translation.x() - cos(rps.cur_loc.angle) * CONFIG_rear_axle_offset;
    rps.robotPosMarker.pose.position.y =
        rps.cur_loc.translation.y() - sin(rps.cur_loc.angle) * CONFIG_rear_axle_offset;
    rps.robotPosMarker.pose.position.z = 0.5 * CONFIG_car_height;
    rps.robotPosMarker.pose.orientation.w = 1.0;
    rps.robotPosMarker.pose.orientation.x = robotQ.x();
    rps.robotPosMarker.pose.orientation.y = robotQ.y();
    rps.robotPosMarker.pose.orientation.z = robotQ.z();
    rps.robotPosMarker.pose.orientation.w = robotQ.w();
  }
}

void Simulator::publishLaser() {
  if (map_.file_name != CONFIG_map_name) {
    map_.Load(CONFIG_map_name);
    drawMap();
  }

  for (size_t i = 0; i < robot_pub_subs_.size(); ++i) {
    auto& rps = robot_pub_subs_[i];
    scanDataMsg.header.stamp = ros::Time::now();
    scanDataMsg.header.frame_id = IndexToPrefix(i) + CONFIG_laser_frame;
    const Vector2f laserRobotLoc(CONFIG_laser_x, CONFIG_laser_y);
    const Vector2f laserLoc =
        rps.cur_loc.translation + Rotation2Df(rps.cur_loc.angle) * laserRobotLoc;

    const int num_rays = static_cast<int>(
        1.0 + (scanDataMsg.angle_max - scanDataMsg.angle_min) /
        scanDataMsg.angle_increment);
    map_.GetPredictedScan(laserLoc,
                          scanDataMsg.range_min,
                          scanDataMsg.range_max,
                          scanDataMsg.angle_min + rps.cur_loc.angle,
                          scanDataMsg.angle_max + rps.cur_loc.angle,
                          num_rays,
                          &scanDataMsg.ranges);
    for (float& r : scanDataMsg.ranges) {
      if (r > scanDataMsg.range_max - 0.1) {
        r = 0;
        continue;
      }
      r = max<float>(0.0, r + CONFIG_laser_stdev * laser_noise_(rng_));
    }

    // TODO Avoid publishing laser twice.
    // Currently publishes once for the visualizer and once for robot
    // requirements.
    rps.laserPublisher.publish(scanDataMsg);
    rps.vizLaserPublisher.publish(scanDataMsg);
  }
}

void Simulator::publishTransform() {
  if (!CONFIG_publish_tfs) {
    return;
  }
  tf::Transform transform;
  tf::Quaternion q;

  for (size_t i = 0; i < robot_pub_subs_.size(); ++i) {
    auto& rps = robot_pub_subs_[i];
    const auto pf = IndexToPrefix(i);
    if(CONFIG_publish_map_to_odom) {
        transform.setOrigin(tf::Vector3(0.0,0.0,0.0));
        transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
        br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map",
        pf + "/odom"));
    }
    transform.setOrigin(tf::Vector3(rps.cur_loc.translation.x(),
          rps.cur_loc.translation.y(), 0.0));
    q.setRPY(0.0, 0.0, rps.cur_loc.angle);
    transform.setRotation(q);
    br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), pf + "/odom",
        pf + "/base_footprint"));

    if(CONFIG_publish_foot_to_base){
        transform.setOrigin(tf::Vector3(0.0 ,0.0, 0.0));
        transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
        br->sendTransform(tf::StampedTransform(transform, ros::Time::now(),
          pf + "/base_footprint", pf + "/base_link"));
    }

    transform.setOrigin(tf::Vector3(CONFIG_laser_x,
          CONFIG_laser_y, CONFIG_laser_z));
    transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1));
    br->sendTransform(tf::StampedTransform(transform, ros::Time::now(),
        pf + "/base_link", pf + "/base_laser"));
  }
}

void Simulator::publishVisualizationMarkers() {
  mapLinesPublisher.publish(lineListMarker);
  objectLinesPublisher.publish(objectLinesMarker);
  for (auto& rps : robot_pub_subs_) {
    rps.posMarkerPublisher.publish(rps.robotPosMarker);
  }
}

// TODO(jaholtz) maybe keep humans seperate from other objects?
void Simulator::publishHumanStates() {
  ut_multirobot_sim::HumanStateArrayMsg human_array_msg;
  for (size_t i = 0; i < objects.size(); ++i) {
    if (objects[i]->GetType() == HUMAN_OBJECT) {
      EntityBase* base = objects[i].get();
      HumanObject* human = static_cast<HumanObject*>(base);

      Vector2f position = human->GetPose().translation;
      double angle = human->GetPose().angle;
      Vector2f trans_vel = human->GetTransVel();
      double rot_vel = human->GetRotVel();

      ut_multirobot_sim::HumanStateMsg human_state_msg;
      human_state_msg.pose.x = position.x();
      human_state_msg.pose.y = position.y();
      human_state_msg.pose.theta = angle;
      human_state_msg.translational_velocity.x = trans_vel.x();
      human_state_msg.translational_velocity.y = trans_vel.y();
      human_state_msg.translational_velocity.z = 0.0;
      human_state_msg.rotational_velocity = rot_vel;

      human_array_msg.human_states.push_back(human_state_msg);
    }
  }
  human_array_msg.header.stamp = ros::Time::now();
  humanStateArrayPublisher.publish(human_array_msg);
}

void Simulator::PublishDoorStates() {
  ut_multirobot_sim::DoorArrayMsg door_array_msg;
  for (size_t i = 0; i < objects.size(); ++i) {
    if (objects[i]->GetType() == DOOR) {
      EntityBase* base = objects[i].get();
      Door* door = static_cast<Door*>(base);

      Vector2f position = door->GetPose().translation;
      double angle = door->GetPose().angle;

      ut_multirobot_sim::DoorStateMsg door_state_msg;
      door_state_msg.pose.x = position.x();
      door_state_msg.pose.y = position.y();
      door_state_msg.pose.theta = angle;
      auto state = door->GetState();
      if (state == door::OPENING || state == door::CLOSING) {
        door_state_msg.doorStatus = 0;
      } else if (state == door::CLOSED) {
        door_state_msg.doorStatus = 1;
      } else {
        door_state_msg.doorStatus = 2;
      }

      door_array_msg.door_states.push_back(door_state_msg);
    }
  }
  door_array_msg.header.stamp = ros::Time::now();
  doorStatePublisher.publish(door_array_msg);
}

void Simulator::update() {
  // Step the motion model forward one time step
  ++sim_step_count;
  sim_time += CONFIG_DT;
  for (auto& rps : robot_pub_subs_) {
    rps.motion_model->Step(CONFIG_DT);
    for (const Line2f& line: rps.motion_model->GetLines()){
      map_.object_lines.push_back(line);
    }

    // Update the simulator with the motion model result.
    rps.cur_loc = rps.motion_model->GetPose();
    rps.vel = rps.motion_model->GetVel();

    // Publishing the ground truth pose
    truePoseMsg.header.stamp = ros::Time::now();
    truePoseMsg.pose.position.x = rps.cur_loc.translation.x();
    truePoseMsg.pose.position.y = rps.cur_loc.translation.y();
    truePoseMsg.pose.position.z = 0;
    truePoseMsg.pose.orientation.w = cos(0.5 * rps.cur_loc.angle);
    truePoseMsg.pose.orientation.z = sin(0.5 * rps.cur_loc.angle);
    truePoseMsg.pose.orientation.x = 0;
    truePoseMsg.pose.orientation.y = 0;
    rps.truePosePublisher.publish(truePoseMsg);
  }
  cout << "Robots Stepped" << endl;
  // motion_model_->Step(CONFIG_DT);
  // ++sim_step_count;
  // sim_time += CONFIG_DT;

  // Update the simulator with the motion model result.
  // cur_loc_ = motion_model_->GetPose();
  // vel_ = motion_model_->GetVel();

  // // Publishing the ground truth pose
  // truePoseMsg.header.stamp = ros::Time::now();
  // truePoseMsg.pose.position.x = cur_loc_.translation.x();
  // truePoseMsg.pose.position.y = cur_loc_.translation.y();
  // truePoseMsg.pose.position.z = 0;
  // truePoseMsg.pose.orientation.w = cos(0.5 * cur_loc_.angle);
  // truePoseMsg.pose.orientation.z = sin(0.5 * cur_loc_.angle);
  // truePoseMsg.pose.orientation.x = 0;
  // truePoseMsg.pose.orientation.y = 0;
  // truePosePublisher.publish(truePoseMsg);

  // Update all map objects and get their lines
  map_.object_lines.clear();
  for (size_t i=0; i < objects.size(); i++){
    objects[i]->Step(CONFIG_DT);
    for (const Line2f& line: objects[i]->GetLines()){
      map_.object_lines.push_back(line);
    }
  }
  this->drawObjects();
  cout << "Map Updated" << endl;
}

string GetMapNameFromFilename(string path) {
  char path_cstring[path.length()];
  strcpy(path_cstring, path.c_str());
  const string file_name(basename(path_cstring));
  static const string suffix = ".vectormap.txt";
  const size_t suffix_len = suffix.length();
  if (file_name.length() > suffix_len &&
      file_name.substr(file_name.length() - suffix_len, suffix_len) == suffix) {
    return file_name.substr(0, file_name.length() - suffix_len);
  }
  return file_name;
}

void Simulator::publishLocalization() {
  for (auto& rps : robot_pub_subs_) {
    localizationMsg.header.stamp = ros::Time::now();
    localizationMsg.map = GetMapNameFromFilename(map_.file_name);
    localizationMsg.pose.x = rps.cur_loc.translation.x();
    localizationMsg.pose.y = rps.cur_loc.translation.y();
    localizationMsg.pose.theta = rps.cur_loc.angle;
    rps.localizationPublisher.publish(localizationMsg);
  }
}

void Simulator::Run() {
  // Simulate time-step.
  update();
  cout << "Updated" << endl;
  //publish odometry and status
  publishOdometry();
  //publish laser rangefinder messages
  publishLaser();
  // publish visualization marker messages
  publishVisualizationMarkers();
  //publish tf
  publishTransform();
  //publish array of human states
  publishHumanStates();
  //publish array of door states
  PublishDoorStates();
  cout << "Published" << endl;

  if (FLAGS_localize) {
    publishLocalization();
  }
}
