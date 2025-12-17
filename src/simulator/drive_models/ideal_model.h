#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "config_reader/config_reader.h"
#include "simulator/drive_models/robot_model.h"

#ifndef SRC_SIMULATOR_IDEAL_MODEL_H_
#define SRC_SIMULATOR_IDEAL_MODEL_H_

namespace ideal {

// Ideal/unconstrained drive model that directly applies commanded velocities
// No acceleration limits, speed limits, or other physical constraints
// Useful for testing and scenarios requiring perfect tracking
class IdealModel : public robot_model::RobotModel {
   private:
    // Interpret geometry_msgs/Twist as direct velocity commands:
    // linear.x, linear.y = translational velocity, angular.z = rotational velocity
    // Commands are applied instantaneously without any limits

    // Standardized drive callback - directly applies commanded velocities
    void DriveCallback(const geometry_msgs::msg::Twist::SharedPtr msg) override;

   public:
    IdealModel() = delete;
    // Initialize with config file (for consistency, though no parameters needed)
    IdealModel(const std::string& config_file);
    ~IdealModel() = default;
    // Update robot state by integrating velocities
    void Step(const double& dt);
};

}  // namespace ideal

#endif  // SRC_SIMULATOR_IDEAL_MODEL_H_
