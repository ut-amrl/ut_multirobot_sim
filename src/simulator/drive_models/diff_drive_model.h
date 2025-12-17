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

    // Config values
    bool invert_linear_vel_cmds_;
    bool invert_angular_vel_cmds_;
    float linear_pos_accel_limit_;   // Linear acceleration limit (forward) [m/s²]
    float linear_neg_accel_limit_;   // Linear acceleration limit (reverse) [m/s²]
    float angular_pos_accel_limit_;  // Angular acceleration limit (CCW) [rad/s²]
    float angular_neg_accel_limit_;  // Angular acceleration limit (CW) [rad/s²]
    float max_angular_vel_;          // Maximum angular velocity [rad/s]
    float max_linear_vel_;           // Maximum linear velocity [m/s]

    float target_linear_vel_;   // Commanded linear velocity [m/s]
    float target_angular_vel_;  // Commanded angular velocity [rad/s]
    double linear_vel_;         // Current linear velocity [m/s]
    double angular_vel_;        // Current angular velocity [rad/s]

    // Standardized drive callback - interprets Twist as diff drive command
    void DriveCallback(const geometry_msgs::msg::Twist::SharedPtr msg) override;

   public:
    DiffDriveModel() = delete;
    // Initialize with config file
    DiffDriveModel(const std::string& config_file);
    ~DiffDriveModel() = default;
    // define Step function for updating
    void Step(const double& dt);
};

}  // namespace diffdrive

#endif  // SRC_SIMULATOR_DIFFDRIVE_MODEL_H_
