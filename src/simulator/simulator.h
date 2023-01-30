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
#include "pedsim_msgs/AgentStates.h"
#include "ros/package.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_datatypes.h"
#include "visualization_msgs/Marker.h"

#include "ut_multirobot_sim/AckermannCurvatureDriveMsg.h"
#include "ut_multirobot_sim/Localization2DMsg.h"
#include "ut_multirobot_sim/DoorControlMsg.h"
#include "ut_multirobot_sim/MarkerColor.h"

#include "std_msgs/Bool.h"
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
using Eigen::Vector2f;
using ut_multirobot_sim::MarkerColor;

class Simulator {
  config_reader::ConfigReader reader_;

  std::vector<std::unique_ptr<EntityBase>> objects;
  ros::NodeHandle _n;

  struct RobotPubSub {
    Pose2Df vel;
    Pose2Df cur_loc;


    ros::Subscriber initSubscriber;
    ros::Publisher odometryTwistPublisher;
    ros::Publisher laserPublisher;
    ros::Publisher vizLaserPublisher;
    ros::Publisher posMarkerPublisher;
    ros::Publisher truePosePublisher;
    ros::Publisher localizationPublisher;
    ros::Publisher velocityArrowPublisher;
    ros::Publisher textPublisher;
    std::unique_ptr<robot_model::RobotModel> motion_model;

    visualization_msgs::Marker robotPosMarker;
    visualization_msgs::Marker robotVelocityArrow;
    visualization_msgs::Marker robotText;
  };
  ros::Subscriber initSubscriber;
  ros::Subscriber doorSubscriber;

  ros::Publisher mapLinesPublisher;
  ros::Publisher objectLinesPublisher;
  ros::Publisher robotLinesPublisher;
  ros::Publisher humanStateArrayPublisher;
  ros::Publisher doorStatePublisher;
  ros::Publisher halt_pub_;
  ros::Publisher go_alone_pub_;
  ros::Publisher follow_pub_;
  ros::Publisher config_pub_;
  vector<tf::TransformBroadcaster *> br;

  sensor_msgs::LaserScan scanDataMsg;
  nav_msgs::Odometry odometryTwistMsg;
  ut_multirobot_sim::Localization2DMsg localizationMsg;

  vector_map::VectorMap map_;

  visualization_msgs::Marker lineListMarker;
  visualization_msgs::Marker objectLinesMarker;
  visualization_msgs::Marker robotLinesMarker;
  visualization_msgs::Marker otherRobotsLinesMarker;


  static const float DT;
  geometry_msgs::PoseStamped truePoseMsg;

  std::default_random_engine rng_;
  std::normal_distribution<float> laser_noise_;

  uint64_t sim_step_count;
  double sim_time;
  std::unique_ptr<robot_model::RobotModel> motion_model_;
  // TODO(jaholtz) Initialize these, possibly all need to be arrays
  // (one per robot)
  bool complete_;
  vector<Pose2Df> goal_pose_;
  vector<Eigen::Vector2f> local_target_;
  // TODO(jaholtz) Open question, what's the best way to handle their not being
  // a next door?
  Pose2Df next_door_pose_;
  int next_door_state_;
  ros::NodeHandle nh_;
  vector<int> action_;
  vector<float> action_vel_x_;
  vector<float> action_vel_y_;
  vector<float> action_vel_angle_;
  vector<int> current_step_;
  vector<int> follow_target_;
  vector<bool> target_locked_;
  vector<human::HumanObject*> target_;
  vector<std::string> robot_texts_;

 private:
  void InitVizMarker(visualization_msgs::Marker &vizMarker, string ns, int id,
                     string type, geometry_msgs::PoseStamped p,
                     geometry_msgs::Point32 scale, double duration,
                     std::vector<float> color);
  void InitSimulatorVizMarkers();
  void DrawMap();
  void DrawObjects();
  void DrawOtherRobots();
    void DoorCallback(const ut_multirobot_sim::DoorControlMsg& msg);
  void InitalLocationCallback(
      const geometry_msgs::PoseWithCovarianceStamped &msg);
  void DriveCallback(const ut_multirobot_sim::AckermannCurvatureDriveMsg &msg);
  void PublishOdometry();
  void PublishLaser();
  void PublishVisualizationMarkers();
  void PublishTransform();
  void PublishLocalization();
  void PublishHumanStates();
  void PublishDoorStates();
  void Update();
  void ClearMarkers();
  void LoadObject(ros::NodeHandle &n);
  void GoAlone(const int& robot_id);
  void Follow(const int& robot_id);
  void Halt();
  void Pass(const int& robot_id);
  void RunAction();
  std::vector<geometry::Line2f> GetRobotLines(const int& robot_id);
  void HaltPub(std_msgs::Bool halt_message);
  vector<human::HumanObject*> GetHumans() const;
  human::HumanObject* FindFollowTarget(const int& robot_index, bool* found);

 public:
  std::vector<RobotPubSub> robot_pub_subs_;
  Simulator() = delete;
  explicit Simulator(const std::string& sim_config);
  ~Simulator();
  bool Init(ros::NodeHandle &n);
  bool Reinit();
  void Run();
  bool Reset();
  void UpdateHumans(const pedsim_msgs::AgentStates& humans);
  double GetSimTime() const { return sim_time; }
  uint64_t GetSimStepCount() const { return sim_step_count; }
  double GetStepSize() const;
  std::vector<Pose2Df> GetRobotPoses() const;
  std::vector<Pose2Df> GetRobotVels() const;
  std::vector<Pose2Df> GetHumanPoses() const;
  std::vector<Pose2Df> GetVisibleHumanPoses(const int& robot_id) const;
  std::vector<Pose2Df> GetVisibleRobotPoses(const int& robot_id) const;

  std::vector<Pose2Df> GetHumanVels() const;
  std::vector<Pose2Df> GetVisibleHumanVels(const int& robot_id) const;
  std::vector<Pose2Df> GetVisibleRobotVels(const int& robot_id) const;

  nav_msgs::Odometry GetOdom(const int& robot_id);
  sensor_msgs::LaserScan GetLaser(const int& robot_id);
  Pose2Df GetGoalPose(const int& robot_id) const;
  Pose2Df GetNextDoorPose() const;
  int GetNextDoorState() const;
  int GetRobotState(const int& robot_id) const;
  int GetFollowTarget(const int& robot_id) const;
  Eigen::Vector2f GetLocalTarget(const int& robot_id) const;
  bool IsComplete(const int& robot_id) const;
  bool GoalReached(const int& robot_id) const;
  bool CheckMapCollision(const int& robot_id) const;
  bool CheckHumanCollision(const int& robot_id) const;
  bool CheckRobotCollision(const int& robot_id) const;
  void SetAction(const int& robot_id, const int& action, const float& action_vel_x, const float& action_vel_y, const float& action_vel_angle);
  void SetMessage(const int& robot_id, const string& message);
  void SetAgentColor(const int& robot_id, const MarkerColor color);
};

namespace simulator {
    robot_model::RobotModel* MakeMotionModel(const std::string& robot_type, ros::NodeHandle& n, const std::string& topic_prefix);
    std::string IndexToPrefix(const size_t index);
};


#endif  // SIMULATOR_H
