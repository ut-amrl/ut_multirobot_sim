#include <random>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "config_reader/config_reader.h"
#include "simulator/drive_models/robot_model.h"

#ifndef SRC_SIMULATOR_OMNIDIRECTIONAL_MODEL_H_
#define SRC_SIMULATOR_OMNIDIRECTIONAL_MODEL_H_

namespace omnidrive {

class OmnidirectionalModel : public robot_model::RobotModel {
   private:
    // Interpret geometry_msgs/Twist as omnidirectional commands:
    // linear.x, linear.y = translational velocity, angular.z = rotational velocity
    std::default_random_engine rng_;
    std::normal_distribution<float> angular_error_;
    config_reader::ConfigReader config_reader_;

    // Standardized drive callback - interprets Twist as omnidirectional command
    void DriveCallback(const geometry_msgs::msg::Twist::SharedPtr msg) override;

   public:
    OmnidirectionalModel() = delete;
    // Initialize with config files
    OmnidirectionalModel(const std::vector<std::string>& config_files);
    ~OmnidirectionalModel() = default;
    // define Step function for updating
    void Step(const double& dt);
};

}  // namespace omnidrive

#endif  // SRC_SIMULATOR_OMNIDIRECTIONAL_MODEL_H_
