#include <math.h>
#include <random>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "config_reader/config_reader.h"
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "simulator/drive_models/robot_model.h"

#ifndef SRC_SIMULATOR_DIFFDRIVE_MODEL_H_
#define SRC_SIMULATOR_DIFFDRIVE_MODEL_H_

namespace diffdrive {

class DiffDriveModel : public robot_model::RobotModel {
   private:
    geometry_msgs::msg::Twist last_cmd_;
    double t_last_cmd_;
    std::default_random_engine rng_;
    std::normal_distribution<float> angular_error_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr drive_subscriber_;
    config_reader::ConfigReader config_reader_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    nav_msgs::msg::Odometry odom_msg_;
    float target_linear_vel_;
    float target_angular_vel_;
    double linear_vel_;
    double angular_vel_;
    geometry_msgs::msg::Quaternion quat_;
    rclcpp::Time last_time_;
    rclcpp::Node::SharedPtr node_;

    // Receives drive callback messages and stores them
    void DriveCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

   public:
    DiffDriveModel() = delete;
    // Intialize a default object reading from a file
    DiffDriveModel(const std::vector<std::string>& config_files,
                   rclcpp::Node::SharedPtr node,
                   const std::string topic_prefix);
    ~DiffDriveModel() = default;
    // define Step function for updating
    void Step(const double& dt);
    void PublishOdom(const float dt);
};

}  // namespace diffdrive

#endif  // SRC_SIMULATOR_DIFFDRIVE_MODEL_H_
