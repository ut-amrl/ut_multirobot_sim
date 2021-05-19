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
 * \file    simulator_main.cpp
 * \brief   A simple simulator.
 * \author  Joydeep Biswas, (C) 2010
 */
//========================================================================

#include <stdio.h>

#include <iostream>

#include "geometry_msgs/PoseStamped.h"
#include "glog/logging.h"
#include "gflags/gflags.h"
#include "pedsim_srvs/StepPedsim.h"
#include "pedsim_srvs/ResetPedsim.h"
#include "ut_multirobot_sim/utmrsStepper.h"
#include "ut_multirobot_sim/utmrsReset.h"
#include "pedsim_msgs/AgentStates.h"
#include "ros/ros.h"
#include "ros/service.h"
#include "std_msgs/Bool.h"
#include "std_srvs/Empty.h"
#include "ut_multirobot_sim/SimulatorStateMsg.h"
#include "ut_multirobot_sim/Pose2Df.h"

#include "shared/util/timer.h"
#include "simulator.h"

using ut_multirobot_sim::SimulatorStateMsg;
using ut_multirobot_sim::utmrsStepper;
using ut_multirobot_sim::utmrsReset;

SimulatorStateMsg sim_state_;
Simulator* simulator_;
bool sim_step_ = false;

DEFINE_string(sim_config, "config/sim_config.lua", "Path to sim config.");
DEFINE_string(scene_config, "config/gdc_gym_gen/scene.xml", "Path to pedsim scene config.");
DEFINE_bool(use_pedsim, false, "Interface with Pedsim for human sim.");
DEFINE_double(speedup_factor, 1.0, "Speedup Simulation");

void SimStartStop(const std_msgs::Bool& msg) {
  sim_state_.sim_state = !sim_state_.sim_state;
  std_srvs::Empty empty;
  if (FLAGS_use_pedsim && !sim_state_.sim_state) {
    ros::service::call("/pedsim_simulator/pause_simulation", empty);
  } else if (FLAGS_use_pedsim) {
    ros::service::call("/pedsim_simulator/unpause_simulation", empty);
  } // if (msg.data) {
    // sim_state_.sim_state = SimulatorStateMsg::SIM_RUNNING;
  // } else {
    // sim_state_.sim_state = SimulatorStateMsg::SIM_STOPPED;
  // }
}

CumulativeFunctionTimer ped_timer("StepPedsim");
void StepPedsim() {
  CumulativeFunctionTimer::Invocation invoke(&ped_timer);
  // Step Pedsim if necessary
  if (FLAGS_use_pedsim) {
    pedsim_srvs::StepPedsim::Request req;
    pedsim_srvs::StepPedsim::Response res;
    req.steps = 1;
    if (ros::service::call("/pedsim_simulator/StepPedsim", req, res)) {
      // Update the simulator with pedsims return
      const pedsim_msgs::AgentStates humans = res.agent_states;
      simulator_->UpdateHumans(humans);
    }
  }
}

CumulativeFunctionTimer sim_timer("StepUTMRS");
void StepSimulator() {
  CumulativeFunctionTimer::Invocation invoke(&sim_timer);
  // Step the Simulator
  simulator_->Run();

  // Disable stepping until a step message is received.
  sim_step_ = false;
}

CumulativeFunctionTimer step_timer("Step");
void Step() {
  CumulativeFunctionTimer::Invocation invoke(&step_timer);
  StepPedsim();
  StepSimulator();

}

void SimStep(const std_msgs::Bool& msg) {
  // In case multiple step commands are received between sim updates, the
  // simulator should try to step at least once.
  sim_step_ = sim_step_ || msg.data;
}

vector<ut_multirobot_sim::Pose2Df> PoseVecToMessage(
    const vector<Pose2Df>& poses) {
  vector<ut_multirobot_sim::Pose2Df> output;
  for (Pose2Df pose : poses) {
    ut_multirobot_sim::Pose2Df pose_msg;
    pose_msg.x = pose.translation.x();
    pose_msg.y = pose.translation.y();
    pose_msg.theta = pose.angle;
    output.push_back(pose_msg);
  }
  return output;
}

bool StepService(utmrsStepper::Request &req,
                 utmrsStepper::Response &res) {
  // Get the action from the request
  // Apply the action (send to the right place?)
  simulator_->SetAction(req.action);
  // Step the simulator as necessary
  Step();

  // Get Poses/Velocities & Convert to Message format
  res.human_poses = PoseVecToMessage(simulator_->GetVisibleHumanPoses(0));
  res.human_vels = PoseVecToMessage(simulator_->GetVisibleHumanVels(0));
  res.robot_poses = PoseVecToMessage(simulator_->GetRobotPoses());
  res.robot_vels = PoseVecToMessage(simulator_->GetRobotVels());


  ut_multirobot_sim::Pose2Df goal_msg;
  Pose2Df goal_pose = simulator_->GetGoalPose();
  goal_msg.x = goal_pose.translation.x();
  goal_msg.y = goal_pose.translation.y();
  goal_msg.theta = goal_pose.angle;
  res.goal_pose = goal_msg;
  res.robot_state = simulator_->GetRobotState();
  res.follow_target = simulator_->GetFollowTarget();
  ut_multirobot_sim::Pose2Df door_msg;
  Pose2Df door_pose = simulator_->GetNextDoorPose();
  door_msg.x = door_pose.translation.x();
  door_msg.y = door_pose.translation.y();
  door_msg.theta = door_pose.angle;
  res.door_pose = door_msg;
  res.door_state = simulator_->GetNextDoorState();
  res.done = simulator_->IsComplete();
  return true;
}

bool ResetService(utmrsReset::Request &req,
                  utmrsReset::Response &res) {
  if (!simulator_->Reset()) {
    return false;
  }
  if (FLAGS_use_pedsim) {
    pedsim_srvs::ResetPedsim::Request req;
    pedsim_srvs::ResetPedsim::Response res;
    // TODO(jaholtz) Get this from elsewhere (scenario generator?)
    req.filename = FLAGS_scene_config;
    ros::service::call("/pedsim_simulator/ResetPedsim", req, res);
  }
  return true;
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  printf("\nUT Multi-Robot Simulator\n\n");

  ros::init(argc, argv, "UT_MultiRobot_Sim");
  ros::NodeHandle n;

  ros::Publisher sim_state_pub = n.advertise<SimulatorStateMsg>(
    "sim_state", 1, true);
  sim_state_.sim_state = SimulatorStateMsg::SIM_STOPPED;
  std_srvs::Empty empty;
  ros::service::call("/pedsim_simulator/pause_simulation", empty);

  ros::Subscriber start_stop_sub = n.subscribe(
    "sim_start_stop", 1, SimStartStop);

  ros::Subscriber step_sub = n.subscribe(
      "sim_step", 1, SimStep);

  ros::ServiceClient pedStep =
      n.serviceClient<pedsim_srvs::StepPedsim>("/pedsim_simulator/StepPedsim", true);

  ros::ServiceServer utmrsStep =
    n.advertiseService("utmrsStepper", StepService);
  ros::ServiceServer utmrsReset =
    n.advertiseService("utmrsReset", ResetService);

  simulator_= new Simulator(FLAGS_sim_config);
  if (!simulator_->Init(n)) {
    return 1;
  }
  // main loop

  RateLoop rate(1.0 / simulator_->GetStepSize());
  while (ros::ok()) {
    ros::spinOnce();
    switch (sim_state_.sim_state) {
      case SimulatorStateMsg::SIM_RUNNING : {
        simulator_->Run();
      } break;
      case SimulatorStateMsg::SIM_STOPPED : {
        // Do nothing unless stepping.
        if (sim_step_) {
          Step();
        }
      } break;
      default: {
        LOG(FATAL) << "Unexpected simulator state: " << sim_state_.sim_state;
      }
    }

    // Publish simulator state.
    sim_state_.sim_step_count = simulator_->GetSimStepCount();
    sim_state_.sim_time = simulator_->GetSimTime();
    sim_state_pub.publish(sim_state_);
    if (sim_state_.sim_state == SimulatorStateMsg::SIM_RUNNING) {
      rate.Sleep();
    }
  }

  printf("closing.\n");
  delete(simulator_);

  return(0);
}
