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
#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/Bool.h"
#include "ut_multirobot_sim/SimulatorStateMsg.h"

#include "shared/util/timer.h"
#include "simulator.h"

using ut_multirobot_sim::SimulatorStateMsg;

SimulatorStateMsg sim_state_;
bool sim_step_ = false;

DEFINE_string(env_config,
              "config/sim_config.lua",
              "Path to environment config.");
DEFINE_string(robot_config,
              "config/ut_jackal_config.lua",
              "Path to robot config.");
DEFINE_string(init_config,
              "config/default_init_config.lua",
              "Path to config for initial state.");
DEFINE_string(maps_dir,
              "",
              "Path to maps directory.");

void SimStartStop(const std_msgs::Bool& msg) {
  if (msg.data) {
    sim_state_.sim_state = SimulatorStateMsg::SIM_RUNNING;
  } else {
    sim_state_.sim_state = SimulatorStateMsg::SIM_STOPPED;
  }
}

void SimStep(const std_msgs::Bool& msg) {
  // In case multiple step commands are received between sim updates, the
  // simulator should step at least once.
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

  if (FLAGS_maps_dir.empty()) {
    FLAGS_maps_dir = ros::package::getPath("amrl_maps");
  }
  CHECK_NE(FLAGS_maps_dir, string(""));
  Simulator simulator(FLAGS_env_config,
                      FLAGS_robot_config,
                      FLAGS_init_config,
                      FLAGS_maps_dir);
  if (!simulator.init(n)) {
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
