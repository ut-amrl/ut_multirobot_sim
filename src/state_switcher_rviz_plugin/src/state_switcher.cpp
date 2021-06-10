#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

#include "state_switcher.hpp"

namespace state_switcher_rviz_plugin {

Follow::Follow() {
  shortcut_key_ = '1';
  state_publisher_ = nh_.advertise<std_msgs::String>("robot_state", 1);
}

void Follow::activate() {
  std_msgs::String msg;
  msg.data = "Follow";
  state_publisher_.publish(msg);
}

void Follow::deactivate() {
}

GoAlone::GoAlone() {
  shortcut_key_ = '2';
  state_publisher_ = nh_.advertise<std_msgs::String>("robot_state", 1);
}

void GoAlone::activate() {
  std_msgs::String msg;
  msg.data = "GoAlone";
  state_publisher_.publish(msg);
}

void GoAlone::deactivate() {
}

Halt::Halt() {
  shortcut_key_ = '3';
  state_publisher_ = nh_.advertise<std_msgs::String>("robot_state", 1);
}

void Halt::activate() {
  std_msgs::String msg;
  msg.data = "Halt";
  state_publisher_.publish(msg);
}

void Halt::deactivate() {
}

Pass::Pass() {
  shortcut_key_ = '4';
  state_publisher_ = nh_.advertise<std_msgs::String>("robot_state", 1);
}

void Pass::activate() {
  std_msgs::String msg;
  msg.data = "Pass";
  state_publisher_.publish(msg);
}

void Pass::deactivate() {
}

Pause::Pause() {
  shortcut_key_ = 'a';
  start_stop_pub_ = nh_.advertise<std_msgs::Bool>("sim_start_stop", 1);
}

void Pause::activate() {
  std_msgs::Bool msg;
  msg.data = true;
  start_stop_pub_.publish(msg);
  deactivate();
}

void Pause::deactivate() {
}

Step::Step() {
  shortcut_key_ = 's';
  step_pub_ = nh_.advertise<std_msgs::Bool>("sim_step", 1);
}

void Step::activate() {
  std_msgs::Bool msg;
  msg.data = true;
  step_pub_.publish(msg);
  deactivate();
}

void Step::deactivate() {
}

} // namespace state_switcher_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(state_switcher_rviz_plugin::Follow, rviz::Tool);
PLUGINLIB_EXPORT_CLASS(state_switcher_rviz_plugin::GoAlone, rviz::Tool);
PLUGINLIB_EXPORT_CLASS(state_switcher_rviz_plugin::Halt, rviz::Tool);
PLUGINLIB_EXPORT_CLASS(state_switcher_rviz_plugin::Pass, rviz::Tool);
PLUGINLIB_EXPORT_CLASS(state_switcher_rviz_plugin::Pause, rviz::Tool);
PLUGINLIB_EXPORT_CLASS(state_switcher_rviz_plugin::Step, rviz::Tool);
