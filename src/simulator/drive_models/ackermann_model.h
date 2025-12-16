#include <random>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "config_reader/config_reader.h"
#include "amrl_msgs/msg/ackermann_curvature_drive_msg.hpp"
#include "simulator/drive_models/robot_model.h"

#ifndef SRC_SIMULATOR_ACKERMANN_MODEL_H_
#define SRC_SIMULATOR_ACKERMANN_MODEL_H_

namespace ackermann {

class AckermannModel : public robot_model::RobotModel {
   private:
    amrl_msgs::msg::AckermannCurvatureDriveMsg last_cmd_;
    double t_last_cmd_;
    std::default_random_engine rng_;
    std::normal_distribution<float> angular_error_;
    rclcpp::Subscription<amrl_msgs::msg::AckermannCurvatureDriveMsg>::SharedPtr drive_subscriber_;
    config_reader::ConfigReader config_reader_;

    // Receives drive callback messages and stores them
    void DriveCallback(const amrl_msgs::msg::AckermannCurvatureDriveMsg::SharedPtr msg);

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
