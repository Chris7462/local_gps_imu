#pragma once

// ros header
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

// Eigen header
#include <Eigen/Geometry>

namespace imu_rotate_node
{

class ImuRotateNode : public rclcpp::Node
{
public:
  ImuRotateNode();
  ~ImuRotateNode() = default;

private:
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  Eigen::Quaterniond init_orientation_inv_;
  bool imu_init_;

  void imu_callback(sensor_msgs::msg::Imu::SharedPtr imu_msg);
};

}
