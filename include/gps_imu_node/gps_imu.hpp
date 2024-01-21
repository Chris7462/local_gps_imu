#pragma once

// ros header
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/transform_broadcaster.h>

// Eigen header
#include <Eigen/Geometry>

// geographicLib header
#include <GeographicLib/LocalCartesian.hpp>


namespace gps_imu_node
{

class GpsImuNode : public rclcpp::Node
{
public:
  GpsImuNode();
  ~GpsImuNode() = default;

private:
  message_filters::Subscriber<sensor_msgs::msg::Imu> sub_imu_;
  message_filters::Subscriber<sensor_msgs::msg::NavSatFix> sub_gps_;

  using policy_t = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Imu, sensor_msgs::msg::NavSatFix>;

  message_filters::Synchronizer<policy_t> sync_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_gps_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  GeographicLib::LocalCartesian geo_converter_;
  Eigen::Isometry3d init_pose_inv_;
  bool oxts_init_;

  void sync_callback(
    const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg,
    const sensor_msgs::msg::NavSatFix::ConstSharedPtr gps_msg);
};

}
