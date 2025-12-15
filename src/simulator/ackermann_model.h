#include <random>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "config_reader/config_reader.h"
#include "ut_multirobot_sim/msg/ackermann_curvature_drive_msg.hpp"
#include "simulator/robot_model.h"

#ifndef SRC_SIMULATOR_ACKERMANN_MODEL_H_
#define SRC_SIMULATOR_ACKERMANN_MODEL_H_

namespace ackermann {

class AckermannModel : public robot_model::RobotModel {
   private:
    ut_multirobot_sim::msg::AckermannCurvatureDriveMsg last_cmd_;
    double t_last_cmd_;
    std::default_random_engine rng_;
    std::normal_distribution<float> angular_error_;
    rclcpp::Subscription<ut_multirobot_sim::msg::AckermannCurvatureDriveMsg>::SharedPtr drive_subscriber_;
    config_reader::ConfigReader config_reader_;

    // Receives drive callback messages and stores them
    void DriveCallback(const ut_multirobot_sim::msg::AckermannCurvatureDriveMsg::SharedPtr msg);

   public:
    AckermannModel() = delete;
    // Intialize a default object reading from a file
    AckermannModel(const std::vector<std::string> &config_file,
                   rclcpp::Node::SharedPtr node);
    ~AckermannModel() = default;
    // define Step function for updating
    void Step(const double &dt);
};

}  // namespace ackermann

#endif  // SRC_SIMULATOR_ACKERMANN_MODEL_H_
