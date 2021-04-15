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

#include "glog/logging.h"
#include "gflags/gflags.h"
#include "pedsim_srvs/StepPedsim.h"
#include "pedsim_msgs/AgentStates.h"
#include "ros/ros.h"
#include "ros/service.h"
#include "std_msgs/Bool.h"
#include "std_srvs/Empty.h"
#include "ut_multirobot_sim/SimulatorStateMsg.h"

#include "shared/util/timer.h"
#include "simulator.h"

using ut_multirobot_sim::SimulatorStateMsg;

SimulatorStateMsg sim_state_;
bool sim_step_ = false;

DEFINE_string(sim_config, "config/sim_config.lua", "Path to sim config.");
DEFINE_bool(use_pedsim, false, "Interface with Pedsim for human sim.");

void SimStartStop(const std_msgs::Bool& msg) {
  sim_state_.sim_state = !sim_state_.sim_state;
  std_srvs::Empty empty;
  if (FLAGS_use_pedsim && !sim_state_.sim_state) {
    ros::service::call("/pedsim_simulator/pause_simulation", empty);
  } else if (FLAGS_use_pedsim) {
    ros::service::call("/pedsim_simulator/unpause_simulation", empty);
  }
  // if (msg.data) {
    // sim_state_.sim_state = SimulatorStateMsg::SIM_RUNNING;
  // } else {
    // sim_state_.sim_state = SimulatorStateMsg::SIM_STOPPED;
  // }
}

void SimStep(const std_msgs::Bool& msg) {
  // In case multiple step commands are received between sim updates, the
  // simulator should try to step at least once.
  sim_step_ = sim_step_ || msg.data;
}

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  printf("\nUT Multi-Robot Simulator\n\n");

  ros::init(argc, argv, "UT_MultiRobot_Sim");
  ros::NodeHandle n;

  ros::Publisher sim_state_pub = n.advertise<SimulatorStateMsg>(
    "sim_state", 1, true);
  sim_state_.sim_state = SimulatorStateMsg::SIM_RUNNING;

  ros::Subscriber start_stop_sub = n.subscribe(
    "sim_start_stop", 1, SimStartStop);

  ros::Subscriber step_sub = n.subscribe(
      "sim_step", 1, SimStep);

  ros::ServiceClient pedStep =
      n.serviceClient<pedsim_srvs::StepPedsim>("/pedsim_simulator/StepPedsim", true);

  // if (FLAGS_use_pedsim) {
    // ros::service::waitForService("/pedsim_simulator/StepPedsim");
  // }

  Simulator simulator(FLAGS_sim_config);
  if (!simulator.Init(n)) {
    return 1;
  }

  // main loop
  RateLoop rate(1.0 / simulator.GetStepSize());
  while (ros::ok()){
    ros::spinOnce();
    switch (sim_state_.sim_state) {
      case SimulatorStateMsg::SIM_RUNNING : {
        simulator.Run();
      } break;
      case SimulatorStateMsg::SIM_STOPPED : {
        // Do nothing unless stepping.
        if (sim_step_) {
          // Step Pedsim if necessary
          if (FLAGS_use_pedsim) {
            pedsim_srvs::StepPedsim::Request req;
            pedsim_srvs::StepPedsim::Response res;
            req.steps = 1;
            if (ros::service::call("/pedsim_simulator/StepPedsim", req, res)) {
              // Update the simulator with pedsims return
              const pedsim_msgs::AgentStates humans = res.agent_states;
              simulator.UpdateHumans(humans);
            }
          }


          // Step the Simulator
          simulator.Run();

          // Disable stepping until a step message is received.
          sim_step_ = false;
        }
      } break;
      default: {
        LOG(FATAL) << "Unexpected simulator state: " << sim_state_.sim_state;
      }
    }

    // Publish simulator state.
    sim_state_.sim_step_count = simulator.GetSimStepCount();
    sim_state_.sim_time = simulator.GetSimTime();
    sim_state_pub.publish(sim_state_);
    rate.Sleep();
  }

  printf("closing.\n");

  return(0);
}
