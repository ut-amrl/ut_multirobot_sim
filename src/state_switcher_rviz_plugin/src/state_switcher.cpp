#include <ros/ros.h>
#include <std_msgs/String.h>

#include "state_switcher.hpp"

namespace state_switcher_rviz_plugin {

Follow::Follow() {
  shortcut_key_ = '1';
  state_publisher_ = nh_.advertise<std_msgs::String>("robot_state", 1);
}

void Follow::activate() {
  std_msgs::String msg;
  msg.data = "FOLLOW";
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
  msg.data = "GO_ALONE";
  state_publisher_.publish(msg);
}

void GoAlone::deactivate() {
}

Stop::Stop() {
  shortcut_key_ = '3';
  state_publisher_ = nh_.advertise<std_msgs::String>("robot_state", 1);
}

void Stop::activate() {
  std_msgs::String msg;
  msg.data = "STOP";
  state_publisher_.publish(msg);
}

void Stop::deactivate() {
}

} // namespace state_switcher_rviz_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(state_switcher_rviz_plugin::Follow, rviz::Tool);
PLUGINLIB_EXPORT_CLASS(state_switcher_rviz_plugin::GoAlone, rviz::Tool);
PLUGINLIB_EXPORT_CLASS(state_switcher_rviz_plugin::Stop, rviz::Tool);