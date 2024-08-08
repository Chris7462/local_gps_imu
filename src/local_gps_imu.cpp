// ROS header
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// local header
#include "local_gps_imu/local_gps_imu.hpp"


namespace local_gps_imu
{

LocalGpsImu::LocalGpsImu()
: Node("local_gps_imu_node"), sync_(policy_t(10), imu_sub_, gps_sub_), init_(false)
{
  rclcpp::QoS qos(10);

  // sync gps and imu msg
  auto rmw_qos_profile = qos.get_rmw_qos_profile();
  imu_sub_.subscribe(this, "kitti/oxts/imu", rmw_qos_profile);
  gps_sub_.subscribe(this, "kitti/oxts/gps/fix", rmw_qos_profile);
  sync_.registerCallback(&LocalGpsImu::sync_callback, this);

  gps_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
    "kitti/oxts/gps_shifted", qos);

  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
    "kitti/oxts/imu_rotated", qos);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
}

void LocalGpsImu::sync_callback(
  const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg,
  const sensor_msgs::msg::NavSatFix::ConstSharedPtr gps_msg)
{
  if (!init_) {
    // convert the GPS coordinate to local coordinate.
    // The first measurement is treated as the inital frame
    geo_converter_.Reset(gps_msg->latitude, gps_msg->longitude, gps_msg->altitude);

    // retive the transform from initial frame to world frame
    tf2::Vector3 t_init;
    t_init.setZero();
    tf2::Quaternion q_init;
    tf2::fromMsg(imu_msg->orientation, q_init);
    q_init.normalize();

    // This is T_{wi}. initial frame ---> world frame
    world_init_trans_.setOrigin(t_init);
    world_init_trans_.setRotation(q_init);

    init_ = true;
  }

  // get current pose in the initial frame (right-handed reference!)
  double x, y, z;
  geo_converter_.Forward(gps_msg->latitude, gps_msg->longitude, gps_msg->altitude, x, y, z);
  tf2::Vector3 t_curr(x, y, z);
  tf2::Quaternion q_curr;
  tf2::fromMsg(imu_msg->orientation, q_curr);

  // This is T_{wc}. current frame ---> world frame
  tf2::Transform world_curr_trans(q_curr, t_curr);

  // This is T_{ic}. current frame ---> initial frame. Now inital frame is the fixed frame.
  tf2::Transform init_curr_trans;
  init_curr_trans.mult(world_init_trans_.inverse(), world_curr_trans);

  // publish shifted gps coordinate in the initial fixed frame
  geometry_msgs::msg::Vector3Stamped gps_shifted_msg;
  gps_shifted_msg.header = gps_msg->header;
  gps_shifted_msg.vector = tf2::toMsg(init_curr_trans.getOrigin());
  gps_pub_->publish(gps_shifted_msg);

  // publish rotated imu orientation in the inital fixed frame
  sensor_msgs::msg::Imu imu_rotated_msg = *imu_msg;
  imu_rotated_msg.orientation = tf2::toMsg(init_curr_trans.getRotation());
  imu_pub_->publish(imu_rotated_msg);

  // publish oxts tf msg
  geometry_msgs::msg::TransformStamped oxts_tf;
  oxts_tf.header.stamp = gps_msg->header.stamp;
  oxts_tf.header.frame_id = "map";
  oxts_tf.child_frame_id = "oxts_link";
  oxts_tf.transform.translation = tf2::toMsg(init_curr_trans.getOrigin());
  oxts_tf.transform.rotation = tf2::toMsg(init_curr_trans.getRotation());

  // Send the transformation
  tf_broadcaster_->sendTransform(oxts_tf);
}

} // namespace local_gps_imu
