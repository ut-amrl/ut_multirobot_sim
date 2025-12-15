#include <random>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "config_reader/config_reader.h"
#include "ut_multirobot_sim/msg/cobot_drive_msg.hpp"
#include "ut_multirobot_sim/msg/cobot_odometry_msg.hpp"
#include "simulator/robot_model.h"

#ifndef SRC_SIMULATOR_OMNIDIRECTIONAL_MODEL_H_
#define SRC_SIMULATOR_OMNIDIRECTIONAL_MODEL_H_

namespace omnidrive {

class OmnidirectionalModel : public robot_model::RobotModel {
   private:
    ut_multirobot_sim::msg::CobotDriveMsg last_cmd_;
    double t_last_cmd_;
    std::default_random_engine rng_;
    std::normal_distribution<float> angular_error_;
    rclcpp::Subscription<ut_multirobot_sim::msg::CobotDriveMsg>::SharedPtr drive_subscriber_;
    config_reader::ConfigReader config_reader_;
    rclcpp::Publisher<ut_multirobot_sim::msg::CobotOdometryMsg>::SharedPtr odom_publisher_;

    // Receives drive callback messages and stores them
    void DriveCallback(const ut_multirobot_sim::msg::CobotDriveMsg::SharedPtr msg);

   public:
    OmnidirectionalModel() = delete;
    // Intialize a default object reading from a file
    OmnidirectionalModel(
        const std::vector<std::string>& config_files, rclcpp::Node::SharedPtr node);
    ~OmnidirectionalModel() = default;
    // define Step function for updating
    void Step(const double& dt);
    void PublishOdom(const float dt);
};

}  // namespace omnidrive

#endif  // SRC_SIMULATOR_OMNIDIRECTIONAL_MODEL_H_
