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
#include <deque>
#ifndef SRC_SIMULATOR_ACKERMANN_MODEL_H_
#define SRC_SIMULATOR_ACKERMANN_MODEL_H_

namespace ackermann {

struct AckermannCmd{
  double t_;
  double vel_;
  double curv_;
};

struct AckermannState{
  Eigen::Vector2f pose_;
  double angle_;
  Eigen::Vector2f vel_;
};

class AckermannModel : public robot_model::RobotModel {
  const bool closed_loop_ = true;
  const double t_act_delay_ = 0.5;
  const double t_obs_delay_ = 0.0;
  const double DT = 0.05;
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
  bool recieved_cmd_;
  // Receives drive callback messages and stores them
  #ifdef AMRL_MSGS
    void DriveCallback(const amrl_msgs::AckermannCurvatureDriveMsg &msg);
  #else
    void DriveCallback(const ut_multirobot_sim::AckermannCurvatureDriveMsg &msg);
  #endif

  // closed loop simulation only
  std::deque<AckermannCmd> cmd_queue;   
  std::deque<AckermannState> state_queue;

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
  // simulated step
  void SStep(double dt);
  void clearRecieved();
  bool isRecieved();
  void updateLastCmd();
  AckermannCmd getCmd(const double& t);
};

}  // namespace ackermann

#endif  // SRC_SIMULATOR_ACKERMANN_MODEL_H_
