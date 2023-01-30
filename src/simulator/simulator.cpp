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
#include <string>

#include "config_reader/macros.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "gflags/gflags.h"

#include "amrl_msgs/Pose2Df.h"
#include "nav_msgs/Odometry.h"
#include "ros/time.h"
#include "sensor_msgs/LaserScan.h"
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
#include "ut_multirobot_sim/HumanControlCommand.h"
#include "ut_multirobot_sim/HumanStateArrayMsg.h"
#include "ut_multirobot_sim/DoorArrayMsg.h"
#include "ut_multirobot_sim/DoorStateMsg.h"
#include "ut_multirobot_sim/Localization2DMsg.h"
#include "ut_multirobot_sim/HumanStateArrayMsg.h"
#include "ut_multirobot_sim/MarkerColor.h"
#include "graph_navigation/socialNavSrv.h"
#include "graph_navigation/socialNavReset.h"
#include "graph_navigation/socialNavReq.h"
#include "graph_navigation/socialNavResp.h"

// #include "ut_multirobot_sim/human_object.h"
#include "vector_map.h"

DEFINE_bool(localize, false, "Publish localization");

// using amrl_msgs::Pose2Df;
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
using std_msgs::Bool;
using omnidrive::OmnidirectionalModel;
using diffdrive::DiffDriveModel;
using vector_map::VectorMap;
using human::HumanObject;
using door::Door;
using door::DoorState;
using ut_multirobot_sim::DoorControlMsg;
using ut_multirobot_sim::HumanStateMsg;
using ut_multirobot_sim::HumanStateArrayMsg;
using ut_multirobot_sim::MarkerColor;

// CONFIG_STRING(init_config_file, "init_config_file");
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
CONFIG_FLOAT(num_humans, "num_humans");
// TF publications
CONFIG_BOOL(publish_tfs, "publish_tfs");
CONFIG_BOOL(publish_map_to_odom, "publish_map_to_odom");
CONFIG_BOOL(publish_foot_to_base, "publish_foot_to_base");
// Observation Controls
CONFIG_BOOL(partially_observable, "partially_observable");

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

CONFIG_INT(max_steps, "max_steps");

CONFIG_STRING(map_name, "map_name");
CONFIG_STRING(nav_map_name, "nav_map_name");
// Initial location
CONFIG_VECTOR3FLIST(start_poses, "start_poses");
CONFIG_VECTOR3FLIST(goal_poses, "goal_poses");
CONFIG_STRINGLIST(short_term_object_config_list, "short_term_object_config_list");
CONFIG_STRING(human_config, "human_config");
CONFIG_STRINGLIST(door_config_list, "door_config");

Simulator::Simulator(const std::string& sim_config) :
    reader_({sim_config}),
    laser_noise_(0, 1),
    sim_step_count(0),
    sim_time(0.0),
    complete_(false),
    goal_pose_({}),
    next_door_pose_(0.0, {0.0, 0.0}),
    //  TODO(jaholtz) set this to the open state (whatever that is)
    next_door_state_(0),
    action_({}),
    current_step_({}),
    robot_texts_({}),
    target_locked_({}) {
  truePoseMsg.header.seq = 0;
  truePoseMsg.header.frame_id = "map";
  if (CONFIG_map_name == "") {
    std::cerr << "Failed to load map from config file '"
              << sim_config << "'" << std::endl;
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

bool Simulator::Reinit(){
  halt_pub_ = _n.advertise<Bool>("/halt_robot", 1);
  go_alone_pub_ = _n.advertise<amrl_msgs::Pose2Df>("/move_base_simple/goal", 1);
  follow_pub_ = _n.advertise<amrl_msgs::Pose2Df>("/nav_override", 1);
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

  robot_pub_subs_.clear();
  local_target_.clear();
  follow_target_.clear();
  target_locked_.clear();
  action_.clear();
  action_vel_x_.clear();
  action_vel_y_.clear();
  action_vel_angle_.clear();
  robot_texts_.clear();
  br.clear();


  // TODO - when making the MakeMotionModel, the CONFIG_robot_types gets reduced to a single item, this
  //  check is no longer valid until MakeMotionalModel is fixed.

  //  if (CONFIG_robot_types.size() != CONFIG_start_poses.size()) {
  //    std::cerr << "Robot type and robot start pose lists are"
  //                 "not the same size!" << std::endl;
  //    return false;
  //  }

  // Create motion model based on robot type
  for (size_t i = 0; i < CONFIG_start_poses.size(); ++i) {
    const auto& robot_type = CONFIG_robot_types.at(0);
    const auto& start_pose = CONFIG_start_poses.at(i);
    const auto pf = IndexToPrefix(i);
    auto* mm = MakeMotionModel(robot_type, _n, pf);
    if (mm == nullptr) {
      return false;
    }
    mm->SetPose(Pose2Df(start_pose.z(), {start_pose.x(), start_pose.y()}));

    goal_pose_.push_back(Pose2Df(0, {CONFIG_goal_poses[i][0], CONFIG_goal_poses[i][1]}));

    local_target_.push_back({0, 0});
    follow_target_.push_back({0});
    target_locked_.push_back({0});
    action_.push_back(0);
    action_vel_x_.push_back(0.);
    action_vel_y_.push_back(0.);
    action_vel_angle_.push_back(0.);
    robot_texts_.push_back("");



    robot_pub_subs_.emplace_back(RobotPubSub());
    auto& rps = robot_pub_subs_.back();
    rps.motion_model = std::unique_ptr<robot_model::RobotModel>(mm);

    rps.initSubscriber = _n.subscribe<ut_multirobot_sim::Localization2DMsg>(
            pf + "/initialpose", 1, [&](const boost::shared_ptr<
                    const ut_multirobot_sim::Localization2DMsg>& msg) {
              const Vector2f loc(msg->pose.x, msg->pose.y);
              const float angle = msg->pose.theta;
              rps.motion_model->SetPose({angle, loc});
            });
    rps.odometryTwistPublisher =
            _n.advertise<nav_msgs::Odometry>(pf + "/odom", 1);
    rps.laserPublisher =
            _n.advertise<sensor_msgs::LaserScan>(pf + CONFIG_laser_topic, 1);
    rps.vizLaserPublisher =
            _n.advertise<sensor_msgs::LaserScan>(pf + "/scan", 1);
    rps.posMarkerPublisher = _n.advertise<visualization_msgs::Marker>(
            0 + "/simulator_visualization", 100);
    rps.truePosePublisher = _n.advertise<geometry_msgs::PoseStamped>(
            0 + "/simulator_true_pose", 100);
    rps.velocityArrowPublisher = _n.advertise<visualization_msgs::Marker>(
            0 + "/simulator_velocity_visualization", 100);
    rps.textPublisher = _n.advertise<visualization_msgs::Marker>(
            0 + "/simulator_text_visualization", 100);

    if (FLAGS_localize) {
      rps.localizationPublisher =
              _n.advertise<ut_multirobot_sim::Localization2DMsg>(
                      pf + "/localization", 1);
      localizationMsg.header.frame_id = "map";
      localizationMsg.header.seq = 0;
    }

    br.push_back(new tf::TransformBroadcaster());

  }

  InitSimulatorVizMarkers();
  DrawMap();

  mapLinesPublisher =
          _n.advertise<visualization_msgs::Marker>("/simulator_visualization", 6);
  objectLinesPublisher =
          _n.advertise<visualization_msgs::Marker>("/simulator_visualization", 6);
  robotLinesPublisher =
          _n.advertise<visualization_msgs::Marker>("/simulator_visualization", 6);

  doorSubscriber =
          _n.subscribe("/door/command", 1, &Simulator::DoorCallback, this);

  humanStateArrayPublisher =
          _n.advertise<ut_multirobot_sim::HumanStateArrayMsg>("/human_states", 1);
  doorStatePublisher =
          _n.advertise<ut_multirobot_sim::DoorArrayMsg>("/door_states", 1);

  this->LoadObject(_n);

  for (size_t i = 0; i < robot_pub_subs_.size(); i++){
    GoAlone(i);
  }
  return true;
}

bool Simulator::Init(ros::NodeHandle& n) {
  _n = n;
  halt_pub_ = n.advertise<Bool>("/halt_robot", 1);
  go_alone_pub_ = n.advertise<amrl_msgs::Pose2Df>("/move_base_simple/goal", 1);
  follow_pub_ = n.advertise<amrl_msgs::Pose2Df>("/nav_override", 1);
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

  // TODO - when making the MakeMotionModel, the CONFIG_robot_types gets reduced to a single item, this
  //  check is no longer valid until MakeMotionalModel is fixed.

  //  if (CONFIG_robot_types.size() != CONFIG_start_poses.size()) {
  //    std::cerr << "Robot type and robot start pose lists are"
  //                 "not the same size!" << std::endl;
  //    return false;
  //  }

  // Create motion model based on robot type
  for (size_t i = 0; i < CONFIG_start_poses.size(); ++i) {
    const auto& robot_type = CONFIG_robot_types.at(0);
    const auto& start_pose = CONFIG_start_poses.at(i);
    const auto pf = IndexToPrefix(i);
    auto* mm = MakeMotionModel(robot_type, n, pf);
    if (mm == nullptr) {
      return false;
    }
    mm->SetPose(Pose2Df(start_pose.z(), {start_pose.x(), start_pose.y()}));

    goal_pose_.push_back(Pose2Df(0, {CONFIG_goal_poses[i][0], CONFIG_goal_poses[i][1]}));

    local_target_.push_back({0, 0});
    follow_target_.push_back({0});
    target_locked_.push_back({0});
    action_.push_back(0);
    action_vel_x_.push_back(0.);
    action_vel_y_.push_back(0.);
    action_vel_angle_.push_back(0.);
    robot_texts_.push_back("");


    robot_pub_subs_.emplace_back(RobotPubSub());
    auto& rps = robot_pub_subs_.back();
    rps.motion_model = std::unique_ptr<robot_model::RobotModel>(mm);

    rps.initSubscriber = n.subscribe<ut_multirobot_sim::Localization2DMsg>(
       pf + "/initialpose", 1, [&](const boost::shared_ptr<
         const ut_multirobot_sim::Localization2DMsg>& msg) {
        const Vector2f loc(msg->pose.x, msg->pose.y);
        const float angle = msg->pose.theta;
        rps.motion_model->SetPose({angle, loc});
      });
    rps.odometryTwistPublisher =
        n.advertise<nav_msgs::Odometry>(pf + "/odom", 1);
    rps.laserPublisher =
        n.advertise<sensor_msgs::LaserScan>(pf + CONFIG_laser_topic, 1);
    rps.vizLaserPublisher =
        n.advertise<sensor_msgs::LaserScan>(pf + "/scan", 1);
    rps.posMarkerPublisher = n.advertise<visualization_msgs::Marker>(
        0 + "/simulator_visualization", 100);
    rps.truePosePublisher = n.advertise<geometry_msgs::PoseStamped>(
        0 + "/simulator_true_pose", 100);
    rps.velocityArrowPublisher = n.advertise<visualization_msgs::Marker>(
            0 + "/simulator_velocity_visualization", 100);
    rps.textPublisher = n.advertise<visualization_msgs::Marker>(
            0 + "/simulator_text_visualization", 100);

    if (FLAGS_localize) {
      rps.localizationPublisher =
              n.advertise<ut_multirobot_sim::Localization2DMsg>(
                      pf + "/localization", 1);
      localizationMsg.header.frame_id = "map";
      localizationMsg.header.seq = 0;
    }

    br.push_back(new tf::TransformBroadcaster());

  }

  InitSimulatorVizMarkers();
  DrawMap();

  mapLinesPublisher =
      n.advertise<visualization_msgs::Marker>("/simulator_visualization", 6);
  objectLinesPublisher =
      n.advertise<visualization_msgs::Marker>("/simulator_visualization", 6);
  robotLinesPublisher =
          n.advertise<visualization_msgs::Marker>("/simulator_visualization", 6);

  doorSubscriber =
      n.subscribe("/door/command", 1, &Simulator::DoorCallback, this);

  humanStateArrayPublisher =
    n.advertise<ut_multirobot_sim::HumanStateArrayMsg>("/human_states", 1);
  doorStatePublisher =
    n.advertise<ut_multirobot_sim::DoorArrayMsg>("/door_states", 1);

  this->LoadObject(n);

  for (size_t i = 0; i < robot_pub_subs_.size(); i++){
    GoAlone(i);
  }
  return true;
}

bool Simulator::Reset() {
  std::cout << "RESETING" << std::endl;

  Simulator::ClearMarkers();
  map_.Load(CONFIG_map_name);

  graph_navigation::socialNavReset::Request all_reqs;
  graph_navigation::socialNavResetReq req = graph_navigation::socialNavResetReq();

  req.number_of_agents = robot_pub_subs_.size();
  req.map_path = CONFIG_nav_map_name;

  all_reqs.reqs.push_back(req);
  graph_navigation::socialNavReset::Response all_res;
  ros::service::call("socialNavReset", all_reqs, all_res);


//  if (CONFIG_start_poses.size() != robot_pub_subs_.size()){
  Simulator::Reinit();
//    return Simulator::Reset();
//  }

  DrawMap();

  std::cout<<"Loading " << CONFIG_map_name << std::endl;

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

  // TODO - when making the MakeMotionModel, the CONFIG_robot_types gets reduced to a single item, this
  //  check is no longer valid until MakeMotionalModel is fixed.

  //  if (CONFIG_robot_types.size() != CONFIG_start_poses.size()) {
  //    std::cerr << "Robot type and robot start pose lists are"
  //                 "not the same size!" << std::endl;
  //    return false;
  //  }

  goal_pose_ = vector<Pose2Df>({});
  action_ = vector<int>({});
  current_step_ = vector<int>({});

  // Update motion model based
  for (size_t i = 0; i < CONFIG_start_poses.size(); ++i) {
    const auto& start_pose = CONFIG_start_poses.at(i);
    const auto pf = IndexToPrefix(i);

    goal_pose_.push_back(Pose2Df(0, {CONFIG_goal_poses[i][0], CONFIG_goal_poses[i][1]}));
    action_.push_back(0);
    action_vel_x_.push_back(0.);
    action_vel_y_.push_back(0.);
    action_vel_angle_.push_back(0.);
    current_step_.push_back(0);

    RobotPubSub* robot = &robot_pub_subs_[i];
    robot->motion_model->SetPose(Pose2Df(start_pose.z(),
                                       {start_pose.x(), start_pose.y()}));

    if (FLAGS_localize) {
        localizationMsg.header.frame_id = "map";
        localizationMsg.header.seq = 0;
      }
  }

  this->LoadObject(nh_);
  Run();
  return true;
}

// TODO(yifeng): Change this into a general way
void Simulator::LoadObject(ros::NodeHandle& nh) {
  objects.clear();

  // human
  for (size_t i = 0; i < CONFIG_num_humans; i++) {
    objects.push_back(
      std::unique_ptr<HumanObject>(new HumanObject({CONFIG_human_config}, i)));
  }

  // door
  for (const string& config_str : CONFIG_door_config_list) {
    objects.push_back(
      std::unique_ptr<Door>(new Door({config_str}))
    );
  }

  // TODO(jaholtz) why is this a separate block and not handled when the
  // human objects are created.
  // for (const std::unique_ptr<EntityBase>& e : objects) {
    // if (e->GetType() == HUMAN_OBJECT) {
      // EntityBase* e_raw = e.get();
      // HumanObject* human = static_cast<HumanObject*>(e_raw);
      // if (human->GetMode() == human::HumanMode::Controlled) {
        // human->InitializeManualControl(nh);
      // }
    // }
  // }
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
void Simulator::InitVizMarker(visualization_msgs::Marker& vizMarker, string ns,
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
  } else if (type == "text") {
    vizMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
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

void Simulator::InitSimulatorVizMarkers() {
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
  InitVizMarker(lineListMarker, "map_lines", 0, "linelist", p, scale, 0.0,
      color);

  for (int i = 0; i < int(robot_pub_subs_.size()); ++i){
    auto& rps = robot_pub_subs_.at(i);
    p.pose.position.z = 0.5 * CONFIG_car_height;
    p.pose.position.x = rps.cur_loc.translation.x();
    p.pose.position.y = rps.cur_loc.translation.y();
    scale.x = CONFIG_car_length;
    scale.y = CONFIG_car_width;
    scale.z = CONFIG_car_height;
    color[0] = 94.0 / 255.0;
    color[1] = 156.0 / 255.0;
    color[2] = 255.0 / 255.0;
    color[3] = 0.8;
    InitVizMarker(rps.robotPosMarker,
                  "robot_position",
                  i+3,
                  "cube",
                  p,
                  scale,
                  0.0,
                  color);
  }

  for (int i = 0; i < int(robot_pub_subs_.size()); ++i){
    auto& rps = robot_pub_subs_.at(i);
    p.pose.position.z = 0.5 * CONFIG_car_height;
    p.pose.position.x = rps.cur_loc.translation.x();
    p.pose.position.y = rps.cur_loc.translation.y();
    scale.x = CONFIG_car_length;
    scale.y = CONFIG_car_width;
    scale.z = CONFIG_car_height;
    color[0] = 255.0 / 255.0;
    color[1] = 165.0 / 255.0;
    color[2] = 0.0 / 255.0;
    color[3] = 0.9;
    InitVizMarker(rps.robotVelocityArrow,
                  "robot_velocity",
                  i+3,
                  "arrow",
                  p,
                  scale,
                  0.0,
                  color);
  }

  for (int i = 0; i < int(robot_pub_subs_.size()); ++i){
    auto& rps = robot_pub_subs_.at(i);
    p.pose.position.z = 0.5 * CONFIG_car_height;
    p.pose.position.x = rps.cur_loc.translation.x();
    p.pose.position.y = rps.cur_loc.translation.y();
    scale.x = CONFIG_car_length;
    scale.y = CONFIG_car_width;
    scale.z = 0.25;
    color[0] = 255.0 / 255.0;
    color[1] = 255.0 / 255.0;
    color[2] = 255.0 / 255.0;
    color[3] = 1.0;
    InitVizMarker(rps.robotText,
                  "robot_text",
                  i+3,
                  "text",
                  p,
                  scale,
                  0.0,
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
  InitVizMarker(objectLinesMarker, "object_lines", 0, "linelist", p, scale,
      0.0, color);

  color[0] = 100.0 / 255.0;
  color[1] = 0.0 / 255.0;
  color[2] = 244.0 / 255.0;
  color[3] = 1.0;
  InitVizMarker(robotLinesMarker, "object_lines", 0, "linelist", p, scale,
                0.0, color);
}

void Simulator::DrawMap() {
  ros_helpers::ClearMarker(&lineListMarker);
  for (const Line2f& l : map_.lines) {
    ros_helpers::DrawEigen2DLine(l.p0, l.p1, &lineListMarker);
  }
}

void Simulator::ClearMarkers() {
  ros_helpers::ClearMarker(&lineListMarker);
  ros_helpers::ClearMarker(&objectLinesMarker);
  ros_helpers::ClearMarker(&robotLinesMarker);

  // TODO: Make these explicit "DELETEALL" markers so you don't need to keep track of them.
  for (auto& rps : robot_pub_subs_) {
    ros_helpers::ClearMarker(&rps.robotPosMarker);
    ros_helpers::ClearMarker(&rps.robotVelocityArrow);
    ros_helpers::ClearMarker(&rps.robotText);
  }
}


void Simulator::DrawObjects() {
  // draw objects
  ros_helpers::ClearMarker(&objectLinesMarker);
  for (const Line2f& l : map_.object_lines) {
    ros_helpers::DrawEigen2DLine(l.p0, l.p1, &objectLinesMarker);
  }
}

void Simulator::DrawOtherRobots() {
  // draw objects
  ros_helpers::ClearMarker(&robotLinesMarker);
  for (const Line2f& l : map_.robot_lines) {
    ros_helpers::DrawEigen2DLine(l.p0, l.p1, &robotLinesMarker);
  }
}

nav_msgs::Odometry Simulator::GetOdom(const int& robot_id) {
  auto& rps = robot_pub_subs_[robot_id];
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
  return odometryTwistMsg;
}

void Simulator::PublishOdometry() {
  int i = 0;
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

    rps.robotPosMarker.pose.position.x =
        rps.cur_loc.translation.x() -
        cos(rps.cur_loc.angle) * CONFIG_rear_axle_offset;
    rps.robotPosMarker.pose.position.y =
        rps.cur_loc.translation.y() -
        sin(rps.cur_loc.angle) * CONFIG_rear_axle_offset;
    rps.robotPosMarker.pose.position.z = 0.5 * CONFIG_car_height;
    rps.robotPosMarker.pose.orientation.w = 1.0;
    rps.robotPosMarker.pose.orientation.x = robotQ.x();
    rps.robotPosMarker.pose.orientation.y = robotQ.y();
    rps.robotPosMarker.pose.orientation.z = robotQ.z();
    rps.robotPosMarker.pose.orientation.w = robotQ.w();


    rps.robotText.pose.position.x =
            rps.cur_loc.translation.x() -
            cos(rps.cur_loc.angle) * CONFIG_rear_axle_offset;
    rps.robotText.pose.position.y =
            rps.cur_loc.translation.y() -
            sin(rps.cur_loc.angle) * CONFIG_rear_axle_offset;
    rps.robotText.pose.position.z = 0.5 * CONFIG_car_height;
    rps.robotText.pose.orientation.w = 1.0;
//    rps.robotText.pose.orientation.x = robotQ.x();
//    rps.robotText.pose.orientation.y = robotQ.y();
//    rps.robotText.pose.orientation.z = robotQ.z();
//    rps.robotText.pose.orientation.w = robotQ.w();

    tf::Quaternion robotvelQ = tf::createQuaternionFromYaw(rps.vel.angle);

    rps.robotVelocityArrow.pose.position.x =
            rps.cur_loc.translation.x() -
            cos(rps.cur_loc.angle) * CONFIG_rear_axle_offset;
    rps.robotVelocityArrow.pose.position.y =
            rps.cur_loc.translation.y() -
            sin(rps.cur_loc.angle) * CONFIG_rear_axle_offset;
    rps.robotVelocityArrow.pose.position.z = 0.5 * CONFIG_car_height;
    rps.robotVelocityArrow.pose.orientation.w = 1.0;
    rps.robotVelocityArrow.pose.orientation.x = robotvelQ.x();
    rps.robotVelocityArrow.pose.orientation.y = robotvelQ.y();
    rps.robotVelocityArrow.pose.orientation.z = robotvelQ.z();
    rps.robotVelocityArrow.pose.orientation.w = robotvelQ.w();


    rps.robotVelocityArrow.scale.x = sqrt(rps.vel.translation.x() + rps.vel.translation.y());

    rps.robotText.text = robot_texts_.at(i);
    i++;
  }
}

sensor_msgs::LaserScan Simulator::GetLaser(const int& robot_id) {
  auto& rps = robot_pub_subs_[robot_id];
  if (map_.file_name != CONFIG_map_name) {
    map_.Load(CONFIG_map_name);
    DrawMap();
  }

  map_.object_lines.clear();
  for (size_t i=0; i < objects.size(); i++){
    objects[i]->Step(CONFIG_DT);

    for (const Line2f& line: objects[i]->GetLines()){
      map_.object_lines.push_back(line);
    }
  }
  this->DrawObjects();

  int i = 0;
  for (auto& rps : robot_pub_subs_) {
    if (i != robot_id){
      for (const Line2f& line: rps.motion_model->GetLines()) {
        map_.object_lines.push_back(line);
      }
    }
    i++;
  }

//  DrawOtherRobots();

  scanDataMsg.header.stamp = ros::Time::now();
  scanDataMsg.header.frame_id = IndexToPrefix(robot_id) + CONFIG_laser_frame;
  const Vector2f laserRobotLoc(CONFIG_laser_x, CONFIG_laser_y);
  const Vector2f laserLoc =
    rps.cur_loc.translation +
    Rotation2Df(rps.cur_loc.angle) * laserRobotLoc;

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

  return scanDataMsg;
}

void Simulator::PublishLaser() {
  if (map_.file_name != CONFIG_map_name) {
    map_.Load(CONFIG_map_name);
    DrawMap();
  }

  for (size_t i = 0; i < robot_pub_subs_.size(); ++i) {
    auto& rps = robot_pub_subs_[i];
    scanDataMsg.header.stamp = ros::Time::now();
    scanDataMsg.header.frame_id = IndexToPrefix(i) + CONFIG_laser_frame;
    const Vector2f laserRobotLoc(CONFIG_laser_x, CONFIG_laser_y);
    const Vector2f laserLoc =
        rps.cur_loc.translation +
        Rotation2Df(rps.cur_loc.angle) * laserRobotLoc;

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

void Simulator::PublishTransform() {
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
        br.at(i)->sendTransform(tf::StampedTransform(transform,
                                               ros::Time::now(), "/map",
                                               pf + "/odom"));
    }

    // Use the first robot to handle the "default" no prefix case.
    // Useful for making this place nicer with single robot cases (and
    // third-party software that assumes the single robot case.)
    if (i == 0 || true) {
        transform.setOrigin(tf::Vector3(0.0,0.0,0.0));
        transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
        br.at(i)->sendTransform(tf::StampedTransform(transform,
                                               ros::Time::now(), pf + "/odom",
                                               "/odom"));
    }
    transform.setOrigin(tf::Vector3(rps.cur_loc.translation.x(),
          rps.cur_loc.translation.y(), 0.0));
    q.setRPY(0.0, 0.0, rps.cur_loc.angle);
    transform.setRotation(q);
    br.at(i)->sendTransform(tf::StampedTransform(transform,
                                           ros::Time::now(),
                                           pf + "/odom",
                                           pf + "/base_footprint"));

    if(CONFIG_publish_foot_to_base){
        transform.setOrigin(tf::Vector3(0.0 ,0.0, 0.0));
        transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
        br.at(i)->sendTransform(tf::StampedTransform(transform, ros::Time::now(),
          pf + "/base_footprint", pf + "/base_link"));
    }

    transform.setOrigin(tf::Vector3(CONFIG_laser_x,
          CONFIG_laser_y, CONFIG_laser_z));
    transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1));
    br.at(i)->sendTransform(tf::StampedTransform(transform, ros::Time::now(),
        pf + "/base_link", pf + CONFIG_laser_frame));
  }
}

void Simulator::PublishVisualizationMarkers() {
  mapLinesPublisher.publish(lineListMarker);
  objectLinesPublisher.publish(objectLinesMarker);
  robotLinesPublisher.publish(robotLinesMarker);
  for (auto& rps : robot_pub_subs_) {
    rps.posMarkerPublisher.publish(rps.robotPosMarker);
    rps.velocityArrowPublisher.publish(rps.robotVelocityArrow);
    rps.textPublisher.publish(rps.robotText);
  }
}

// TODO(jaholtz) maybe keep humans seperate from other objects?
void Simulator::PublishHumanStates() {
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
      // Updating next door with the first door in the list, only works
      // in single door environments.
      // TODO(jaholtz) Identify the next door on each robot's path.
      next_door_pose_ = door->GetPose();
      next_door_state_ = door_state_msg.doorStatus;

      door_array_msg.door_states.push_back(door_state_msg);
    }
  }
  door_array_msg.header.stamp = ros::Time::now();
  doorStatePublisher.publish(door_array_msg);
}

void Simulator::Update() {
  // Step the motion model forward one time step
  ++sim_step_count;
  sim_time += CONFIG_DT;
  for (auto& rps : robot_pub_subs_) {
    rps.motion_model->Step(CONFIG_DT);
    for (const Line2f& line: rps.motion_model->GetLines()) {
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

  // Update all map objects and get their lines
  map_.object_lines.clear();
  for (size_t i=0; i < objects.size(); i++){
    objects[i]->Step(CONFIG_DT);

    for (const Line2f& line: objects[i]->GetLines()){
      map_.object_lines.push_back(line);
    }
  }

  this->DrawObjects();
}
std::vector<geometry::Line2f> Simulator::GetRobotLines(const int& robot_id){
  const auto& rps = robot_pub_subs_[robot_id];

  const float r = 1;
  const int num_segments = 20;

  const float angle_increment = 2 * M_PI / num_segments;

  std::vector<geometry::Line2f> template_lines_ = {};

  Eigen::Vector2f v0(rps.cur_loc.translation.x() + r, rps.cur_loc.translation.y());
  Eigen::Vector2f v1;
  const float eps = 0.001;
  for (int i = 1; i < num_segments; i++) {
    v1 = Eigen::Rotation2Df(angle_increment * i) * Eigen::Vector2f(rps.cur_loc.translation.x() + r, rps.cur_loc.translation.y());

    Eigen::Vector2f eps_vec = (v1 - v0).normalized() * eps;
    template_lines_.push_back(geometry::Line2f(v0 + eps_vec, v1 - eps_vec));
    v0 = v1;
  }
  template_lines_.push_back(geometry::Line2f(v1, Eigen::Vector2f(rps.cur_loc.translation.x() + r, rps.cur_loc.translation.y())));
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

void Simulator::PublishLocalization() {
  for (auto& rps : robot_pub_subs_) {
    localizationMsg.header.stamp = ros::Time::now();
    localizationMsg.map = GetMapNameFromFilename(map_.file_name);
    localizationMsg.pose.x = rps.cur_loc.translation.x();
    localizationMsg.pose.y = rps.cur_loc.translation.y();
    localizationMsg.pose.theta = rps.cur_loc.angle;
    rps.localizationPublisher.publish(localizationMsg);
  }
}

void Simulator::UpdateHumans(const pedsim_msgs::AgentStates& humans) {
  const vector<pedsim_msgs::AgentState> human_list = humans.agent_states;
  int human_id = 0;
  // Iterate throught the objects and find the humans
  for (size_t i = 0; i < objects.size(); ++i) {
    if (objects[i]->GetType() == HUMAN_OBJECT) {
      // Cast the object as a human
      EntityBase* e_raw = objects[i].get();
      HumanObject* human = static_cast<HumanObject*>(e_raw);
      // Get the corresponding agent state from pedsim
      pedsim_msgs::AgentState agent_state = human_list[human_id];
      ut_multirobot_sim::HumanControlCommand command;
      command.header.frame_id = agent_state.header.frame_id;
      command.header.stamp = ros::Time::now();
      command.translational_velocity = agent_state.twist.linear;
      command.rotational_velocity = 0.0;
      command.pose = agent_state.pose.position;

      human->ManualControlCb(command);
      human_id++;
    }
  }
}

vector<Pose2Df> Simulator::GetRobotPoses() const {
  vector<Pose2Df> output;
  for (auto& rps : robot_pub_subs_) {
    output.push_back(rps.cur_loc);
  }
  return output;
}

vector<Pose2Df> Simulator::GetRobotVels() const {
  vector<Pose2Df> output;
  for (auto& rps : robot_pub_subs_) {
    output.push_back(rps.vel);
  }
  return output;
}

//vector<Pose2Df> Simulator::GetVisibleRobotPoses(const int& robot_id) const {
//
//}

vector<Pose2Df> Simulator::GetVisibleRobotPoses(const int& robot_id) const {
  vector<Pose2Df> output;
  const Vector2f robot_pose = robot_pub_subs_[robot_id].cur_loc.translation;
  const float robot_angle = robot_pub_subs_[robot_id].cur_loc.angle;
  const Pose2Df zero_pose(0, {0, 0});
  ut_multirobot_sim::HumanStateArrayMsg human_array_msg;
  for (size_t i = 0; i < robot_pub_subs_.size(); ++i) {
    if (int(i) == robot_id){
      continue;
    }

    const Pose2Df other_robot_pose = robot_pub_subs_[i].cur_loc;

    if (map_.Intersects(robot_pose, other_robot_pose.translation) && CONFIG_partially_observable) {
      output.push_back(zero_pose);
    } else {
      Eigen::Rotation2Df rotation(-robot_angle);
      const Vector2f local_robot =
              rotation * (other_robot_pose.translation - robot_pose);
      const Pose2Df local_pose(other_robot_pose.angle, local_robot);
      output.push_back(local_pose);
    }
  }
  return output;
}

vector<Pose2Df> Simulator::GetVisibleHumanPoses(const int& robot_id) const {
  vector<Pose2Df> output;
  const Vector2f robot_pose = robot_pub_subs_[robot_id].cur_loc.translation;
  const float robot_angle = robot_pub_subs_[robot_id].cur_loc.angle;
  const Pose2Df zero_pose(0, {0, 0});
  ut_multirobot_sim::HumanStateArrayMsg human_array_msg;
  for (size_t i = 0; i < objects.size(); ++i) {
    if (objects[i]->GetType() == HUMAN_OBJECT) {
      EntityBase* base = objects[i].get();
      HumanObject* human = static_cast<HumanObject*>(base);
      const Pose2Df human_pose = human->GetPose();
      if (map_.Intersects(robot_pose, human_pose.translation) && CONFIG_partially_observable) {
        output.push_back(zero_pose);
      } else {
        Eigen::Rotation2Df rotation(-robot_angle);
        const Vector2f local_human =
                rotation * (human_pose.translation - robot_pose);
        const Pose2Df local_pose(human_pose.angle, local_human);
        output.push_back(local_pose);
      }
    }
  }
  return output;
}

vector<Pose2Df> Simulator::GetHumanPoses() const {
  vector<Pose2Df> output;
  ut_multirobot_sim::HumanStateArrayMsg human_array_msg;
  for (size_t i = 0; i < objects.size(); ++i) {
    if (objects[i]->GetType() == HUMAN_OBJECT) {
      EntityBase* base = objects[i].get();
      HumanObject* human = static_cast<HumanObject*>(base);
      output.push_back(human->GetPose());
    }
  }
  return output;
}

vector<Pose2Df> Simulator::GetVisibleHumanVels(const int& robot_id) const {
  vector<Pose2Df> output;
  const Vector2f robot_pose = robot_pub_subs_[robot_id].cur_loc.translation;
  const Vector2f robot_vel = robot_pub_subs_[robot_id].vel.translation;
  const float robot_angle = robot_pub_subs_[robot_id].cur_loc.angle;
  const Pose2Df zero_pose(0, {0, 0});
  ut_multirobot_sim::HumanStateArrayMsg human_array_msg;
  for (size_t i = 0; i < objects.size(); ++i) {
    if (objects[i]->GetType() == HUMAN_OBJECT) {
      EntityBase* base = objects[i].get();
      HumanObject* human = static_cast<HumanObject*>(base);
      const Pose2Df human_pose = human->GetPose();
      if (map_.Intersects(robot_pose, human_pose.translation) && CONFIG_partially_observable) {
          output.push_back(zero_pose);
      } else {
        Eigen::Rotation2Df rotation(-robot_angle);
        const Vector2f local_human =
            rotation * (human->GetTransVel()) - robot_vel;
        output.push_back({static_cast<float>(human->GetRotVel()), local_human});
      }
    }
  }
  return output;
}

vector<Pose2Df> Simulator::GetVisibleRobotVels(const int& robot_id) const {
  vector<Pose2Df> output;
  const Vector2f robot_pose = robot_pub_subs_[robot_id].cur_loc.translation;
  const Vector2f robot_vel = robot_pub_subs_[robot_id].vel.translation;
  const float robot_angle = robot_pub_subs_[robot_id].cur_loc.angle;
  const Pose2Df zero_pose(0, {0, 0});
  ut_multirobot_sim::HumanStateArrayMsg human_array_msg;
  for (size_t i = 0; i < robot_pub_subs_.size(); ++i) {

    if (int(i) == robot_id){
      continue;
    }

    const Pose2Df other_robot_pose = robot_pub_subs_[i].cur_loc;
    if (map_.Intersects(robot_pose, other_robot_pose.translation) && CONFIG_partially_observable) {
      output.push_back(zero_pose);
    } else {
      Eigen::Rotation2Df rotation(-robot_angle);
      const Vector2f local_human =
              rotation * robot_pub_subs_[i].vel.translation - robot_vel;
      output.push_back({static_cast<float>(robot_pub_subs_[i].vel.angle), local_human});
    }
  }
  return output;
}

vector<Pose2Df> Simulator::GetHumanVels() const {
  vector<Pose2Df> output;
  ut_multirobot_sim::HumanStateArrayMsg human_array_msg;
  for (size_t i = 0; i < objects.size(); ++i) {
    if (objects[i]->GetType() == HUMAN_OBJECT) {
      EntityBase* base = objects[i].get();
      HumanObject* human = static_cast<HumanObject*>(base);
      output.push_back({static_cast<float>(human->GetRotVel()), human->GetTransVel()});
    }
  }
  return output;
}

Pose2Df Simulator::GetGoalPose(const int& robot_id) const {
  return goal_pose_.at(robot_id);
}

Pose2Df Simulator::GetNextDoorPose() const {
  return next_door_pose_;
}

int Simulator::GetNextDoorState() const {
  return next_door_state_;
}

bool Simulator::IsComplete(const int& robot_id) const {
  const RobotPubSub* robot = &robot_pub_subs_[robot_id];
  const float distance =
      (robot->cur_loc.translation - goal_pose_.at(robot_id).translation).norm();
  const float goal_threshold_ = 0.5;
  const bool timeout = current_step_.at(robot_id) > CONFIG_max_steps;
  return distance < goal_threshold_ || timeout;
}

bool Simulator::CheckHumanCollision(const int& robot_id) const {
  const RobotPubSub* robot = &robot_pub_subs_[robot_id];

  // Check for collision with a human
  const vector<HumanObject*> humans = GetHumans();
  const Vector2f robot_pose = robot->cur_loc.translation;

  for (size_t i = 0; i < humans.size(); i++) {
    const auto human = humans[i];
    const Vector2f h_pose(human->GetPose().translation.x(),
                          human->GetPose().translation.y());
    //
    const float kCollisionDist = 0.4;
    const Vector2f diff = robot_pose - h_pose;
    if (diff.norm() < kCollisionDist) {
      return true;
    }
  }

  return false;
}

bool Simulator::CheckRobotCollision(const int& robot_id) const {
  const RobotPubSub* robot = &robot_pub_subs_[robot_id];

  // Check for collision with a human
  const Vector2f robot_pose = robot->cur_loc.translation;

  for (size_t i = 0; i < robot_pub_subs_.size(); i++) {
    if (int(i) == robot_id) {
      continue;
    }

    const Vector2f r_pose(robot_pub_subs_[i].cur_loc.translation.x(),
                          robot_pub_subs_[i].cur_loc.translation.y());
    //
    const float kCollisionDist = 0.4;
    const Vector2f diff = robot_pose - r_pose;
    if (diff.norm() < kCollisionDist) {
      return true;
    }
  }

  return false;
}

bool Simulator::CheckMapCollision(const int& robot_id) const {
  const RobotPubSub* robot = &robot_pub_subs_[robot_id];
  // Check for collision with the map
  const Vector2f robot_length({0.4, 0.0});
  const Eigen::Rotation2Df rot(robot->cur_loc.angle);
  if (map_.Intersects(robot->cur_loc.translation,
                      robot->cur_loc.translation + rot * robot_length)) {
    return true;
  }

  return false;
}

bool Simulator::GoalReached(const int& robot_id) const {
  const RobotPubSub* robot = &robot_pub_subs_[robot_id];
  const float distance =
      (robot->cur_loc.translation - goal_pose_.at(robot_id).translation).norm();
  const float goal_threshold_ = 0.5;
  return distance < goal_threshold_;
}

CumulativeFunctionTimer pub_timer("Publishing Halt");
void Simulator::HaltPub(Bool halt_message) {
  CumulativeFunctionTimer::Invocation invoke(&pub_timer);
  halt_pub_.publish(halt_message);
}

CumulativeFunctionTimer halt_timer("Halt");
void Simulator::Halt() {
  CumulativeFunctionTimer::Invocation invoke(&halt_timer);
  Bool halt_message;
  halt_message.data = true;
  HaltPub(halt_message);
  // TODO - publish the robot id
  for (auto& rps : robot_pub_subs_) {
      rps.motion_model->SetVel({0, {0, 0}});
  }
}

CumulativeFunctionTimer ga_timer("GoAlone");
void Simulator::GoAlone(const int& robot_id) {
  CumulativeFunctionTimer::Invocation invoke(&ga_timer);
  amrl_msgs::Pose2Df target_message;
  target_message.x = goal_pose_.at(robot_id).translation.x();
  target_message.y = goal_pose_.at(robot_id).translation.y();
  target_message.theta = goal_pose_.at(robot_id).angle;
  // TODO - publish the robot id
  std::cout << "Publish GoAlone message" << std::endl;
  go_alone_pub_.publish(target_message);
}

vector<HumanObject*> Simulator::GetHumans() const {
  vector<HumanObject*> output;
  for (size_t i = 0; i < objects.size(); ++i) {
    if (objects[i]->GetType() == HUMAN_OBJECT) {
      EntityBase* base = objects[i].get();
      HumanObject* human = static_cast<HumanObject*>(base);
      output.push_back(human);
    }
  }
  return output;
}

HumanObject* GetClosest(const vector<HumanObject*> humans,
                        const Vector2f pose,
                        const vector<int> indices,
                        int* index) {
  float best_dist = 9999;
  HumanObject* best_human = nullptr;
  int best_index = 0;
  int count = 0;
  for (HumanObject* human : humans) {
    const Vector2f h_pose = human->GetPose().translation;
    const Vector2f diff = h_pose - pose;
    const float dist = diff.norm();
    if (dist < best_dist) {
      best_dist = dist;
      best_human = human;
      best_index = count;
    }
    count++;
  }
  *index = best_index;
  return best_human;
}

HumanObject* Simulator::FindFollowTarget(const int& robot_index,
                                         bool* found) {
  // Order is front_left, front, front_right
  vector<HumanObject*> left;
  vector<HumanObject*> front_left;
  vector<HumanObject*> front;
  vector<HumanObject*> front_right;
  vector<HumanObject*> right;
  vector<int> indices;
  // todo(jaholtz) Consider if we need to shrink or grow this margin.
  const float kRobotLength = 0.5;
  const float kLowerLeft = DegToRad(90.0);
  const float kUpperLeft = DegToRad(15.0);
  const float kLowerRight = DegToRad(270.0);
  const float kUpperRight = DegToRad(345.0);

  const vector<HumanObject*> humans = GetHumans();
  const RobotPubSub* robot = &robot_pub_subs_[robot_index];
  const Vector2f pose = robot->cur_loc.translation;
  const float theta = robot->cur_loc.angle;

  for (size_t i = 0; i < humans.size(); i++) {
    HumanObject* human = humans[i];
    const Vector2f h_pose(human->GetPose().translation.x(),
                          human->GetPose().translation.y());
    const bool intersects = map_.Intersects(pose, h_pose);
    if (intersects) {
      break;
    }
    // Transform the pose to robot reference frame
    const Vector2f diff = h_pose - pose;
    Eigen::Rotation2Df rot(-theta);
    const Vector2f transformed = rot * diff;
    const float angle = math_util::AngleMod(geometry::Angle(diff) - theta);
    if (transformed.x() > kRobotLength) {
      if (angle < kLowerLeft && angle > kUpperLeft) {
        front_left.push_back(human);
      } else if (angle > kLowerRight && angle < kUpperRight) {
        front_right.push_back(human);
      } else if (angle < kUpperLeft || angle > kUpperRight) {
        front.push_back(human);
        indices.push_back(i);
      }
    } else if (transformed.x() > 0) {
      if (angle < kLowerLeft && angle > kUpperLeft) {
        left.push_back(human);
      } else if (angle > kLowerRight && angle < kUpperRight) {
        right.push_back(human);
      }
    }
  }
  *found = true;
  if (front.size() < 1) {
    *found = false;
  }
  return GetClosest(front, pose, indices, &follow_target_.at(robot_index));
}

CumulativeFunctionTimer follow_timer("Follow");
void Simulator::Follow(const int& robot_id) {
  CumulativeFunctionTimer::Invocation invoke(&follow_timer);
  const float kFollowDist = 1.5;
  // TODO(jaholtz) Currently assumes only one robot.
  if (!target_locked_.at(robot_id)) {
    bool found = true;
    target_[robot_id] = FindFollowTarget(robot_id, &found);
    if (!found) {
      Halt();
      return;
    }
    target_locked_[robot_id] = true;
  }
  const RobotPubSub* robot = &robot_pub_subs_[robot_id];
  const Vector2f pose = robot->cur_loc.translation;
  const Vector2f h_pose = target_.at(robot_id)->GetPose().translation;
  const Vector2f towards_bot = pose - h_pose;
  const Vector2f target_pose = h_pose + kFollowDist * towards_bot.normalized();
  amrl_msgs::Pose2Df follow_msg;
  follow_msg.x = target_pose.x();
  follow_msg.y = target_pose.y();

  // TODO - Publish the robot id
  follow_pub_.publish(follow_msg);
}

CumulativeFunctionTimer pass_timer("Pass");
void Simulator::Pass(const int& robot_id) {
  CumulativeFunctionTimer::Invocation invoke(&pass_timer);
  const float kLeadDist = 1.5;
  bool found = true;
  HumanObject* target = FindFollowTarget(robot_id, &found);
  if (!found) {
    Halt();
    return;
  }
  const Vector2f h_pose = target->GetPose().translation;
  const Vector2f target_vel = target->GetTransVel();
  const Vector2f target_pose = h_pose + kLeadDist * target_vel.normalized();
  amrl_msgs::Pose2Df follow_msg;
  follow_msg.x = target_pose.x();
  follow_msg.y = target_pose.y();
  // TODO - publish robot id
  follow_pub_.publish(follow_msg);
}

void Simulator::SetAction(const int& robot_id, const int& action, const float& action_vel_x, const float& action_vel_y, const float& action_vel_angle) {
  if (action_.at(robot_id) != action) {
    target_locked_.at(robot_id) = false;
  }
  action_.at(robot_id) = action;
  action_vel_x_.at(robot_id) = action_vel_x;
  action_vel_y_.at(robot_id) = action_vel_y;
  action_vel_angle_.at(robot_id) = action_vel_angle;
}

void Simulator::SetMessage(const int& robot_id, const string& message){
  robot_texts_.at(robot_id) = message;
}

void Simulator::SetAgentColor(const int &robot_id, const MarkerColor color) {
  if (color.r < 0.){
    robot_pub_subs_.at(robot_id).robotPosMarker.color.r = 94.0 / 255.0;
    robot_pub_subs_.at(robot_id).robotPosMarker.color.g = 156.0 / 255.0;
    robot_pub_subs_.at(robot_id).robotPosMarker.color.b = 255.0 / 255.0;
    return;
  }
  robot_pub_subs_.at(robot_id).robotPosMarker.color.r = color.r / 255.0;
  robot_pub_subs_.at(robot_id).robotPosMarker.color.g = color.g / 255.0;
  robot_pub_subs_.at(robot_id).robotPosMarker.color.b = color.b / 255.0;
}


int Simulator::GetFollowTarget(const int& robot_id) const {
  if (action_.at(robot_id) == 2 || action_.at(robot_id) == 3) {
    return follow_target_.at(robot_id);
  }
  return -1;
}

Vector2f Simulator::GetLocalTarget(const int& robot_id) const {
  return local_target_.at(robot_id);
}

int Simulator::GetRobotState(const int& robot_id) const {
  return action_.at(robot_id);
}

vector<geometry_msgs::Pose2D> PoseVecToGeomMessage(
    const vector<Pose2Df>& poses) {
  vector<geometry_msgs::Pose2D> output;
  for (Pose2Df pose : poses) {
    geometry_msgs::Pose2D pose_msg;
    pose_msg.x = pose.translation.x();
    pose_msg.y = pose.translation.y();
    pose_msg.theta = pose.angle;
    output.push_back(pose_msg);
  }
  return output;
}

void Simulator::RunAction() {
  graph_navigation::socialNavSrv::Request all_reqs;
  graph_navigation::socialNavSrv::Response all_res;
  // Assuming we're working with the first robot
  for (size_t i = 0; i < robot_pub_subs_.size(); i++) {
    graph_navigation::socialNavReq req = graph_navigation::socialNavReq();
    RobotPubSub *robot = &robot_pub_subs_[i];
    req.action = action_.at(i);
    req.action_vel_x = action_vel_x_.at(i);
    req.action_vel_y = action_vel_y_.at(i);
    req.action_vel_angle = action_vel_angle_.at(i);

    req.loc.x = robot->cur_loc.translation.x();
    req.loc.y = robot->cur_loc.translation.y();
    req.loc.theta = robot->cur_loc.angle;
    req.odom = GetOdom(i);
    req.laser = GetLaser(i);
    req.goal_pose.x = goal_pose_.at(i).translation.x();
    req.goal_pose.y = goal_pose_.at(i).translation.y();
    req.goal_pose.theta = goal_pose_.at(i).angle;
    double time = sim_time;
    req.time = time;
    req.human_poses = PoseVecToGeomMessage(GetVisibleHumanPoses(i));
    req.human_vels = PoseVecToGeomMessage(GetVisibleHumanVels(i));

    all_reqs.nav_reqs.push_back(req);
  }

  ros::service::call("socialNavSrv", all_reqs, all_res);

  for (size_t i = 0; i < robot_pub_subs_.size(); i++) {
    RobotPubSub *robot = &robot_pub_subs_[i];

    if (action_.at(i) == 1) {
      robot->motion_model->SetVel({0, {0, 0}});
    } else {
      if (all_res.nav_resps.size() <= i){
        continue;
      }
      // Update the simulator with navigation result
      robot->motion_model->SetCmd(all_res.nav_resps.at(i).cmd_vel, all_res.nav_resps.at(i).cmd_curve);
      if (all_res.nav_resps.at(i).cmd_vel == 0.0) {
        robot->motion_model->SetVel({0, {0, 0}});
      }
      local_target_[i] = {all_res.nav_resps.at(i).local_target.x, all_res.nav_resps.at(i).local_target.y};
    }
  }
}

void Simulator::Run() {
  // Run Action
  for (size_t i = 0; i < robot_pub_subs_.size(); i++) {
    current_step_[i]++;
  }

  RunAction();
  // Simulate time-step.
  Update();
  // //publish odometry and status
  PublishOdometry();
  // //publish laser rangefinder messages
  PublishLaser();
  // // publish visualization marker messages
  PublishVisualizationMarkers();
  // //publish tf
  PublishTransform();
  // //publish array of human states
  PublishHumanStates();
  // //publish array of door states
  PublishDoorStates();

  if (FLAGS_localize) {
    PublishLocalization();
  }
}
