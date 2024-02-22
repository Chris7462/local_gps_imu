// ROS header
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

// local header
#include "gps_imu_node/gps_imu.hpp"


namespace gps_imu_node
{

GpsImuNode::GpsImuNode()
: Node("gps_imu_node"), sync_(policy_t(10), imu_sub_, gps_sub_), oxts_init_(false)
{
  rclcpp::QoS qos(10);

  // sync gps and imu msg
  auto rmw_qos_profile = qos.get_rmw_qos_profile();
  imu_sub_.subscribe(this, "kitti/oxts/imu", rmw_qos_profile);
  gps_sub_.subscribe(this, "kitti/oxts/gps/fix", rmw_qos_profile);
  sync_.registerCallback(&GpsImuNode::sync_callback, this);

  pub_gps_ = create_publisher<sensor_msgs::msg::NavSatFix>(
    "kitti/oxts/gps_shifted", qos);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void GpsImuNode::sync_callback(
  const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg,
  const sensor_msgs::msg::NavSatFix::ConstSharedPtr gps_msg)
{
  if (!oxts_init_) {
    // use the first quaternion as (0,0,0,0) for orientation
    Eigen::Quaterniond init_orientation;
    tf2::fromMsg(imu_msg->orientation, init_orientation);
    init_orientation_inv_ = init_orientation.inverse();

    // use the first gps point as map (0,0,0)
    geo_converter_.Reset(gps_msg->latitude, gps_msg->longitude, gps_msg->altitude);

    oxts_init_ = true;
  }

  Eigen::Quaterniond orientation;
  tf2::fromMsg(imu_msg->orientation, orientation);
  orientation = init_orientation_inv_ * orientation;

  // pose information from gps_msg
  double lat, lon, alt;
  geo_converter_.Forward(gps_msg->latitude, gps_msg->longitude, gps_msg->altitude, lat, lon, alt);

  // rotate the pose with init_orientation_
  Eigen::Vector3d pose = init_orientation_inv_ * Eigen::Vector3d(lat, lon, alt);

  // publish gps shifted
  sensor_msgs::msg::NavSatFix gps_shifted_msg = *gps_msg;
  gps_shifted_msg.latitude = pose.x();
  gps_shifted_msg.longitude = pose.y();
  gps_shifted_msg.altitude = pose.z();
  pub_gps_->publish(gps_shifted_msg);

  // publish oxts tf msg
  geometry_msgs::msg::TransformStamped oxts_tf;
  oxts_tf.header.stamp = gps_msg->header.stamp;
  oxts_tf.header.frame_id = "map";
  oxts_tf.child_frame_id = "oxts_link";
  oxts_tf.transform.translation = tf2::toMsg2(pose);
  oxts_tf.transform.rotation = tf2::toMsg(orientation);

  // Send the transformation
  tf_broadcaster_->sendTransform(oxts_tf);
}

} // namespace gps_imu_node
