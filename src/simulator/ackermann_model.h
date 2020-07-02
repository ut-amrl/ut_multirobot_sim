// comment out to use ut_multirobot_sim's message.
#define AMRL_MSGS

#include <random>
#include <string>
#include "config_reader/config_reader.h"
#ifdef AMRL_MSGS
  #include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#else
  #include "ut_multirobot_sim/AckermannCurvatureDriveMsg.h"
#endif
#include "ros/ros.h"
#include "simulator/robot_model.h"

#ifndef SRC_SIMULATOR_ACKERMANN_MODEL_H_
#define SRC_SIMULATOR_ACKERMANN_MODEL_H_

namespace ackermann {

class AckermannModel : public robot_model::RobotModel {
 private:
  #ifdef AMRL_MSGS
    amrl_msgs::AckermannCurvatureDriveMsg last_cmd_;
  #else
    ut_multirobot_sim::AckermannCurvatureDriveMsg last_cmd_;
  #endif
  double t_last_cmd_;
  std::default_random_engine rng_;
  std::normal_distribution<float> angular_error_;
  ros::Subscriber drive_subscriber_;
  config_reader::ConfigReader config_reader_;

  // Receives drive callback messages and stores them
  #ifdef AMRL_MSGS
    void DriveCallback(const amrl_msgs::AckermannCurvatureDriveMsg &msg);
  #else
    void DriveCallback(const ut_multirobot_sim::AckermannCurvatureDriveMsg &msg);
  #endif
  // Initialize associated template lines (shape of robot)
  void SetTemplateLines(const float r, const int num_segments);
  void Transform();
 public:
  AckermannModel() = delete;
  // Intialize a default object reading from a file
  AckermannModel(const std::vector<std::string> &config_file,
                 ros::NodeHandle *n, std::string prefix);
  ~AckermannModel() = default;
  // define Step function for updating
  void Step(const double &dt);
};

}  // namespace ackermann

#endif  // SRC_SIMULATOR_ACKERMANN_MODEL_H_
