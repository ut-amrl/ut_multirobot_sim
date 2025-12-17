#include <random>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "config_reader/config_reader.h"
#include "simulator/drive_models/robot_model.h"

#ifndef SRC_SIMULATOR_ACKERMANN_MODEL_H_
#define SRC_SIMULATOR_ACKERMANN_MODEL_H_

namespace ackermann {

class AckermannModel : public robot_model::RobotModel {
   private:
    // Interpret geometry_msgs/Twist as desired velocities:
    // linear.x = velocity (m/s), angular.z = angular velocity (rad/s)
    // Then compute curvature = angular_velocity / linear_velocity internally
    std::default_random_engine rng_;
    std::normal_distribution<float> angular_error_;

    // Config values
    float min_turn_radius_;
    float max_accel_;
    float max_speed_;
    float angular_bias_;
    float angular_error_rate_;

    // Standardized drive callback - interprets Twist as Ackermann command
    void DriveCallback(const geometry_msgs::msg::Twist::SharedPtr msg) override;

   public:
    AckermannModel() = delete;
    // Initialize with config file
    AckermannModel(const std::string& config_file);
    ~AckermannModel() = default;
    // define Step function for updating
    void Step(const double& dt);
};

}  // namespace ackermann

#endif  // SRC_SIMULATOR_ACKERMANN_MODEL_H_
