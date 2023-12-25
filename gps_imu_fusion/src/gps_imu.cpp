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
  if (!oxts_init_) {
    // use the first imu orientation as (0,0,0,0)
    Eigen::Quaterniond init_rot;
    tf2::fromMsg(imu_msg->orientation, init_rot);
    initial_rotation_inv_ = init_rot.toRotationMatrix().transpose();

    // use the first gps point as map (0,0,0)
    geo_converter_.Reset(gps_msg->latitude, gps_msg->longitude, gps_msg->altitude);

    oxts_init_ = true;
  }

  // orientation information from imu_msg
  Eigen::Quaterniond imu_orientation;
  tf2::fromMsg(imu_msg->orientation, imu_orientation);

  // pose information from gps_msg
  double lat, lon, alt;
  geo_converter_.Forward(gps_msg->latitude, gps_msg->longitude, gps_msg->altitude, lat, lon, alt);

  // rotate the pose with init_rotation_
  Eigen::Vector3d pose = initial_rotation_inv_ * Eigen::Vector3d(lat, lon, alt);  // p = R * p

  // rotate orientation and convert to quaternion
  Eigen::Quaterniond orientation(initial_rotation_inv_ * imu_orientation.toRotationMatrix());

  // publish tf msg
  geometry_msgs::msg::TransformStamped gps_imu_tf;
  gps_imu_tf.header.stamp = rclcpp::Node::now();
  gps_imu_tf.header.frame_id = "map";
  gps_imu_tf.child_frame_id = "base_link";
  gps_imu_tf.transform.translation = tf2::toMsg2(pose);
  gps_imu_tf.transform.rotation = tf2::toMsg(orientation);

  // Send the transformation
  tf_broadcaster_->sendTransform(gps_imu_tf);
};

}
