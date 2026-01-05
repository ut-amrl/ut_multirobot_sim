#include <string>

#include <geometry_msgs/msg/twist.hpp>

#include "simulator/drive_models/robot_model.h"

#ifndef SRC_SIMULATOR_OMNIDIRECTIONAL_MODEL_H_
#define SRC_SIMULATOR_OMNIDIRECTIONAL_MODEL_H_

namespace omnidrive {

class OmnidirectionalModel : public robot_model::RobotModel {
   private:
    // Interpret geometry_msgs/Twist as omnidirectional commands:
    // linear.x, linear.y = translational velocity, angular.z = rotational velocity

    // Config values
    float max_accel_;        // Maximum linear acceleration [m/s²]
    float max_angle_accel_;  // Maximum angular acceleration [rad/s²]
    float max_speed_;        // Maximum linear velocity [m/s]
    float max_angle_vel_;    // Maximum angular velocity [rad/s]
    bool apply_limits_;      // Enable/disable speed/accel limits

    // Standardized drive callback - interprets Twist as omnidirectional command
    void DriveCallback(const geometry_msgs::msg::Twist::SharedPtr msg) override;

   public:
    OmnidirectionalModel() = delete;
    // Initialize with config file
    OmnidirectionalModel(const std::string& config_file);
    ~OmnidirectionalModel() = default;
    // define Step function for updating
    void Step(const double& dt);
};

}  // namespace omnidrive

#endif  // SRC_SIMULATOR_OMNIDIRECTIONAL_MODEL_H_
