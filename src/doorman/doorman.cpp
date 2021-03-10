#include <eigen3/Eigen/Eigen>
#include <gflags/gflags.h>
#include <stdexcept>
#include <sys/signal.h>

#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "ut_multirobot_sim/DoorControlMsg.h"
#include "ut_multirobot_sim/HumanStateMsg.h"
#include "ut_multirobot_sim/HumanStateArrayMsg.h"

using Eigen::Vector2f;
using ut_multirobot_sim::DoorControlMsg;
using ut_multirobot_sim::HumanStateArrayMsg;
using ut_multirobot_sim::HumanStateMsg;

DEFINE_string(ros_topic, "/door/command", "Topic for HumanControlCommands");

const Vector2f door_center_loc(27.98, 9.62);

Vector2f robot_loc = {0, 0};
void RobotLocationCallback(const nav_msgs::Odometry& msg) {
  robot_loc.x() = msg.pose.pose.position.x;
  robot_loc.y() = msg.pose.pose.position.y;
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "doorman");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<DoorControlMsg>(FLAGS_ros_topic, 1);
  ros::Subscriber sub = nh.subscribe("/odom", 1, &RobotLocationCallback);

  DoorControlMsg dcm;
  while (ros::ok()) {
    ros::spinOnce();
    if ((door_center_loc - robot_loc).norm() < 3) {
      dcm.command = dcm.OPENING;
      pub.publish(dcm);
    } else {
      dcm.command = dcm.CLOSING;
      pub.publish(dcm);
    }
  }
}