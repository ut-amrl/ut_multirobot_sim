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
// comment out to use ut_multirobot_sim's message.
#define AMRL_MSGS

#include <stdio.h>
#include <iostream>
#include <memory>
#include <random>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
#include "ros/package.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"
#include "visualization_msgs/Marker.h"

#ifdef AMRL_MSGS
  #include "amrl_msgs/Localization2DMsg.h"
#else
  #include "ut_multirobot_sim/Localization2DMsg.h"
  //#include "ut_multirobot_sim/AckermannCurvatureDriveMsg.h"
#endif


#include "shared/math/geometry.h"
#include "shared/util/timer.h"
#include "simulator/vector_map.h"
#include "config_reader/config_reader.h"

#include "entity_base.h"
#include "human_object.h"
#include "robot_model.h"
#include "short_term_object.h"

#ifndef SIMULATOR_H
#define SIMULATOR_H

using namespace std;
using pose_2d::Pose2Df;

class Simulator {
  config_reader::ConfigReader reader_;
  config_reader::ConfigReader init_config_reader_;

  // tenporary variables
  Pose2Df vel_;
  Pose2Df cur_loc_;

  std::vector<std::unique_ptr<EntityBase>> objects;

  std::vector<ros::Subscriber> initSubscribers_;

  std::vector<ros::Publisher> odometryTwistPublishers_;
  std::vector<ros::Publisher> laserPublishers_;
  std::vector<ros::Publisher> viz_laser_publishers_;
  ros::Publisher mapLinesPublisher_;
  ros::Publisher posMarkerPublisher_;
  ros::Publisher objectLinesPublisher_;
  ros::Publisher truePosePublisher_;
  std::vector<ros::Publisher> localizationPublishers_;
  tf::TransformBroadcaster *br_;

  sensor_msgs::LaserScan scanDataMsg_;
  nav_msgs::Odometry odometryTwistMsg_;
  #ifdef AMRL_MSGS
    amrl_msgs::Localization2DMsg localizationMsg_;
  #else
    ut_multirobot_sim::Localization2DMsg localizationMsg_;
  #endif

  vector_map::VectorMap map_;

  visualization_msgs::Marker lineListMarker_;
  std::vector<visualization_msgs::Marker> robotPosMarkers_;
  visualization_msgs::Marker objectLinesMarker_;

  static const float DT;
  geometry_msgs::PoseStamped truePoseMsg_;

  std::default_random_engine rng_;
  std::normal_distribution<float> laser_noise_;

  // multi-robot variables
  std::vector<std::unique_ptr<robot_model::RobotModel>> motion_models_;
  int robot_number_;
  //std::vector<ros::Publisher> truePosePublishers;
  // topic prefixes
  std::vector<std::string> topic_prefixs_;
  std::vector<std::string> robot_types_;

  std::vector<Pose2Df> cur_locs_;
  // object lines associated with each vector
  std::vector<std::vector<geometry::Line2f>> motion_models_lines_;
 private:
  void initVizMarker(visualization_msgs::Marker &vizMarker, string ns, int id,
                     string type, geometry_msgs::PoseStamped p,
                     geometry_msgs::Point32 scale, double duration,
                     std::vector<float> color);
  void initSimulatorVizMarkers();
  void drawMap();
  void drawObjects();
  void InitalLocationCallback(
      const geometry_msgs::PoseWithCovarianceStamped &msg);
  // void DriveCallback(const ut_multirobot_sim::AckermannCurvatureDriveMsg &msg);
  void publishOdometry(int cur_robot_number);
  void publishLaser(int cur_robot_number);
  void publishVisualizationMarkers(int cur_robot_number);
  void publishTransform(int cur_robot_number);
  void update(int cur_robot_number);
  void updateLocation(int cur_robot_number);
  void updateSimulatorLines();
  void loadObject();


  // TODO: figure out higher order function
  void InitalLocationCallbackMulti(
    const geometry_msgs::PoseWithCovarianceStamped &msg, int robot_num);
  void InitalLocationCallbackRobot0(
    const geometry_msgs::PoseWithCovarianceStamped &msg);
  void InitalLocationCallbackRobot1(
    const geometry_msgs::PoseWithCovarianceStamped &msg);
  void InitalLocationCallbackRobot2(
    const geometry_msgs::PoseWithCovarianceStamped &msg);
  void InitalLocationCallbackRobot3(
    const geometry_msgs::PoseWithCovarianceStamped &msg);
  void InitalLocationCallbackRobot4(
    const geometry_msgs::PoseWithCovarianceStamped &msg);
  void InitalLocationCallbackRobot5(
    const geometry_msgs::PoseWithCovarianceStamped &msg);
  void InitalLocationCallbackRobot6(
    const geometry_msgs::PoseWithCovarianceStamped &msg);
  void InitalLocationCallbackRobot7(
    const geometry_msgs::PoseWithCovarianceStamped &msg);

 public:
  Simulator() = delete;
  explicit Simulator(const std::string& sim_config);
  ~Simulator();
  bool init(ros::NodeHandle &n);
  void Run();
};
#endif  // SIMULATOR_H
