// ROS header
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

// Eigen header
#include <Eigen/Geometry>

// local header
#include "gps_imu_node/gps_imu.hpp"


namespace gps_imu_node
{

GpsImuNode::GpsImuNode()
  : Node("gps_imu_node"), oxts_init_(false), sync_(policy_t(10), sub_imu_, sub_gps_)
{
  rclcpp::QoS qos(10);

  // sync gps and imu msg
  auto rmw_qos_profile = qos.get_rmw_qos_profile();
  sub_gps_.subscribe(this, "kitti/nav_sat_fix", rmw_qos_profile);
  sub_imu_.subscribe(this, "kitti/imu", rmw_qos_profile);
  sync_.registerCallback(&GpsImuNode::sync_callback, this);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void GpsImuNode::sync_callback(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg,
  const sensor_msgs::msg::NavSatFix::ConstSharedPtr gps_msg)
{
  // use the first gps signal as (0,0,0)
  if (!oxts_init_) {
    geo_converter_.Reset(gps_msg->latitude, gps_msg->longitude, gps_msg->altitude);
    Eigen::Quaterniond init_rot;
    tf2::fromMsg(imu_msg->orientation, init_rot);
    initial_rotation_ = init_rot.toRotationMatrix().transpose();
    oxts_init_ = true;
  }

  // orientation information from imu_msg
  Eigen::Quaterniond imu_orientation(imu_msg->orientation.w,
    imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z);

  // pose information from gps_msg
  double lat, lon, alt;
  geo_converter_.Forward(gps_msg->latitude, gps_msg->longitude, gps_msg->altitude, lat, lon, alt);

  // rotate the pose with init_rotation_
  Eigen::Vector3d pose = initial_rotation_ * Eigen::Vector3d(lat, lon, alt);

  // rotate orientation and convert to quaternion
  Eigen::Quaterniond orientation(imu_orientation.toRotationMatrix() * initial_rotation_);

  // publish tf msg
  geometry_msgs::msg::TransformStamped gps_imu_tf;
  gps_imu_tf.header.stamp = rclcpp::Node::now();
  gps_imu_tf.header.frame_id = "map";
  gps_imu_tf.child_frame_id = "base_link";
  gps_imu_tf.transform.translation.x = pose.x();
  gps_imu_tf.transform.translation.y = pose.y();
  gps_imu_tf.transform.translation.z = pose.z();
  gps_imu_tf.transform.rotation.x = orientation.x();
  gps_imu_tf.transform.rotation.y = orientation.y();
  gps_imu_tf.transform.rotation.z = orientation.z();
  gps_imu_tf.transform.rotation.w = orientation.w();

  // Send the transformation
  tf_broadcaster_->sendTransform(gps_imu_tf);
};

}
