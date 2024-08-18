#pragma once

// ros header
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

// local created ROS message
#include "kitti_msgs/msg/geo_plane_point.hpp"


namespace local_gps_imu
{

class LocalGpsImu : public rclcpp::Node
{
public:
  LocalGpsImu();
  ~LocalGpsImu() = default;

private:
  message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub_;
  message_filters::Subscriber<sensor_msgs::msg::NavSatFix> gps_sub_;

  using policy_t = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Imu, sensor_msgs::msg::NavSatFix>;

  message_filters::Synchronizer<policy_t> sync_;

  rclcpp::Publisher<kitti_msgs::msg::GeoPlanePoint>::SharedPtr gps_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  //GeographicLib::LocalCartesian geo_converter_;
  bool init_;

  tf2::Transform new_world_world_trans_;
  tf2::Transform base_oxts_trans_;
  tf2::Transform oxts_base_trans_;

  void sync_callback(
    const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg,
    const sensor_msgs::msg::NavSatFix::ConstSharedPtr gps_msg);

  void wait_for_tf();
};

} // namespace local_gps_imu
