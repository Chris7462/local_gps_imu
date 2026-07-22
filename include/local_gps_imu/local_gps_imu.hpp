#pragma once

// std header
#include <optional>

// ros header
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/synchronizer.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

// local created ROS message
#include "av_msgs/msg/geo_plane_point.hpp"


namespace local_gps_imu
{

class LocalGpsImu : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for LocalGpsImu node
   */
  LocalGpsImu();

  /**
   * @brief Destructor for LocalGpsImu node
   */
  ~LocalGpsImu() = default;

private:
  /**
   * @brief Initialize node parameters with validation
   * @return true if initialization successful, false otherwise
   */
  bool initialize_parameters();

  /**
   * @brief Initialize ROS2 publishers, subscribers, TF, and message filters
   * @return true if initialization successful, false otherwise
   */
  bool initialize_ros_components();

  /**
   * @brief Synchronized callback for incoming IMU, GPS, and velocity messages
   * @param imu_msg Incoming IMU message
   * @param gps_msg Incoming NavSatFix message
   * @param vel_msg Incoming velocity message
   */
  void sync_callback(
    const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg,
    const sensor_msgs::msg::NavSatFix::ConstSharedPtr gps_msg,
    const geometry_msgs::msg::TwistStamped::ConstSharedPtr vel_msg);

  /**
   * @brief Block until the static base_frame -> oxts_frame transform is available
   */
  void wait_for_tf();

private:
  // Fixed frame names (tied to the KITTI URDF, not meant to be reconfigured at launch)
  const std::string base_frame_id_ = "base_link";
  const std::string oxts_frame_id_ = "oxts_link";

  // ROS2 parameters: topics
  std::string imu_input_topic_;
  std::string gps_input_topic_;
  std::string vel_input_topic_;
  std::string gps_output_topic_;
  std::string imu_output_topic_;
  std::string vel_output_topic_;

  // ROS2 parameters: queues / timeouts
  int queue_size_;
  int sync_queue_size_;
  double tf_wait_timeout_;

  // Message filter subscribers
  message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub_;
  message_filters::Subscriber<sensor_msgs::msg::NavSatFix> gps_sub_;
  message_filters::Subscriber<geometry_msgs::msg::TwistStamped> vel_sub_;

  using policy_t = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Imu, sensor_msgs::msg::NavSatFix, geometry_msgs::msg::TwistStamped>;

  std::shared_ptr<message_filters::Synchronizer<policy_t>> sync_;

  rclcpp::Publisher<av_msgs::msg::GeoPlanePoint>::SharedPtr gps_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // T_{base, oxts} and its inverse T_{oxts, base}. Both depend only on the
  // static base_link -> oxts_link transform, so they're computed once in
  // wait_for_tf() rather than lazily on the first sensor message.
  tf2::Transform base_oxts_trans_;
  tf2::Transform oxts_base_trans_;

  // T_{new_world, world}. Only knowable once the first synchronized message
  // arrives, so it's computed lazily on first use in sync_callback().
  std::optional<tf2::Transform> new_world_world_trans_;
};

} // namespace local_gps_imu
