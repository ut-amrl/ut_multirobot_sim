#pragma once

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <rviz/tool.h>
#endif

namespace state_switcher_rviz_plugin {

class Follow : public rviz::Tool {
Q_OBJECT
public:
  Follow();
  ros::NodeHandle nh_;
  ros::Publisher state_publisher_;

  // virtual void onInitialize();
  virtual void activate();
  virtual void deactivate();
};

class GoAlone : public rviz::Tool {
Q_OBJECT
public:
  GoAlone();
  ros::NodeHandle nh_;
  ros::Publisher state_publisher_;

  // virtual void onInitialize();
  virtual void activate();
  virtual void deactivate();
};

class Stop : public rviz::Tool {
Q_OBJECT
public:
  Stop();
  ros::NodeHandle nh_;
  ros::Publisher state_publisher_;

  // virtual void onInitialize();
  virtual void activate();
  virtual void deactivate();
};

} // namespace state_switcher_rviz_plugin