// ROS header
#include <tf2_eigen/tf2_eigen.hpp>

// local header
#include "gps_imu_node/imu_rotate.hpp"


namespace imu_rotate_node
{

ImuRotateNode::ImuRotateNode()
: Node("imu_rotate_node"), imu_init_(false)
{
  rclcpp::QoS qos(10);

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "kitti/oxts/imu", qos,
    std::bind(&ImuRotateNode::imu_callback, this, std::placeholders::_1));

  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
    "kitti/oxts/imu_rotated", qos);
}

void ImuRotateNode::imu_callback(sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
  if (!imu_init_) {
    // use the first quaternion as (0,0,0,0) for orientation
    Eigen::Quaterniond init_orientation;
    tf2::fromMsg(imu_msg->orientation, init_orientation);
    init_orientation_inv_ = init_orientation.inverse();
    imu_init_ = true;
  }

  // rotate the orientation with init_orientation_inv_
  Eigen::Quaterniond orientation;
  tf2::fromMsg(imu_msg->orientation, orientation);
  orientation = init_orientation_inv_ * orientation;

  // publish imu rotated
  sensor_msgs::msg::Imu imu_rotated_msg = *imu_msg;
  imu_rotated_msg.orientation = tf2::toMsg(orientation);
  imu_pub_->publish(imu_rotated_msg);
}

} // namespace imu_rotate_node
