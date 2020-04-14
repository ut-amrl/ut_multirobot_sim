#include "config_reader/config_reader.h"
#include "f1tenth_simulator/AckermannCurvatureDriveMsg.h"
#include "ros/ros.h"
#include "simulator/robot_model.h"
#include <random>
#include <string>

#ifndef SRC_SIMULATOR_ACKERMANN_MODEL_H_
#define SRC_SIMULATOR_ACKERMANN_MODEL_H_

namespace robot_model {

class AckermannModel : public RobotModel {
private:
  f1tenth_simulator::AckermannCurvatureDriveMsg last_cmd_;
  double t_last_cmd_;
  std::default_random_engine rng_;
  std::normal_distribution<float> angular_error_;
  ros::Subscriber drive_subscriber_;
  config_reader::ConfigReader config_reader_;

  // Receives drive callback messages and stores them
  void DriveCallback(const f1tenth_simulator::AckermannCurvatureDriveMsg &msg);

public:
  AckermannModel() = delete;
  // Intialize a default object reading from a file
  AckermannModel(const std::string &config_file, ros::NodeHandle *n);
  ~AckermannModel() = default;
  // define Step function for updating
  void Step(const double &dt);
};

} // namespace robot_model

#endif // SRC_SIMULATOR_ACKERMANN_MODEL_H_
