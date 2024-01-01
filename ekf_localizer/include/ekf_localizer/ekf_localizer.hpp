#pragma once

// C++ header
#include <queue>
#include <mutex>

// ros header
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Transform.h>

// geographicLib header
#include <GeographicLib/LocalCartesian.hpp>

// local header
#include "kalman_filter/system_model.hpp"
#include "kalman_filter/gps_measurement_model.hpp"
#include "kalman_filter/imu_measurement_model.hpp"
#include "kalman_filter/vel_measurement_model.hpp"
#include "kalman_filter/extended_kalman_filter.hpp"


namespace ekf_localizer
{

class EkfLocalizer : public rclcpp::Node
{
public:
  EkfLocalizer();
  ~EkfLocalizer() = default;

private:
  double freq_;
  bool gps_init_;

  double alt_;  // for publish purpose
  double pitch_;  // for publish purpose
  double roll_; // for publish purpose

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  tf2::Transform odom_base_link_trans_;

  std::queue<sensor_msgs::msg::Imu::SharedPtr> imu_buff_;
  std::queue<sensor_msgs::msg::NavSatFix::SharedPtr> gps_buff_;
  std::queue<geometry_msgs::msg::TwistStamped::SharedPtr> vel_buff_;

  GeographicLib::LocalCartesian geo_converter_;

  std::mutex mtx_;

  void run_ekf();

  kalman::SystemModel sys_;
  kalman::ImuMeasurementModel imu_model_;
  kalman::GpsMeasurementModel gps_model_;
  kalman::VelMeasurementModel vel_model_;
  kalman::ExtendedKalmanFilter ekf_;
};

} // namespace ekf_localizer
