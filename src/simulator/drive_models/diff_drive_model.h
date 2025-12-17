#include <math.h>
#include <random>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "config_reader/config_reader.h"
#include "simulator/drive_models/robot_model.h"

#ifndef SRC_SIMULATOR_DIFFDRIVE_MODEL_H_
#define SRC_SIMULATOR_DIFFDRIVE_MODEL_H_

namespace diffdrive {

class DiffDriveModel : public robot_model::RobotModel {
   private:
    // Interpret geometry_msgs/Twist as differential drive commands:
    // linear.x = forward velocity, angular.z = rotational velocity
    std::default_random_engine rng_;
    std::normal_distribution<float> angular_error_;
    config_reader::ConfigReader config_reader_;
    float target_linear_vel_;
    float target_angular_vel_;
    double linear_vel_;
    double angular_vel_;
    rclcpp::Time last_time_;

    // Standardized drive callback - interprets Twist as diff drive command
    void DriveCallback(const geometry_msgs::msg::Twist::SharedPtr msg) override;

   public:
    DiffDriveModel() = delete;
    // Initialize with config files
    DiffDriveModel(const std::vector<std::string>& config_files);
    ~DiffDriveModel() = default;
    // define Step function for updating
    void Step(const double& dt);
};

}  // namespace diffdrive

#endif  // SRC_SIMULATOR_DIFFDRIVE_MODEL_H_
