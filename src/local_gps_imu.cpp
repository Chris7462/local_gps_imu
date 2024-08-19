// ROS header
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// geographicLib header
#include <GeographicLib/UTMUPS.hpp>

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

  gps_pub_ = create_publisher<kitti_msgs::msg::GeoPlanePoint>(
    "kitti/vehicle/gps_local", qos);

  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
    "kitti/vehicle/imu_local", qos);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  wait_for_tf();
}

void LocalGpsImu::sync_callback(
  const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg,
  const sensor_msgs::msg::NavSatFix::ConstSharedPtr gps_msg)
{
  // dummy variables
  int zone;
  bool northp;

  if (!init_) {
    // set the first frame as the new world frame.
    double init_x, init_y, init_z;
    GeographicLib::UTMUPS::Forward(gps_msg->latitude, gps_msg->longitude,
      zone, northp, init_x, init_y);
    init_z = gps_msg->altitude;

    tf2::Vector3 t_init(init_x, init_y, init_z);
    tf2::Quaternion q_init;
    tf2::fromMsg(imu_msg->orientation, q_init);
    q_init.normalize();
    tf2::Transform world_oxts_trans(q_init, t_init);

    // T_{new_world, world}
    new_world_world_trans_.mult(base_oxts_trans_, world_oxts_trans.inverse());
    oxts_base_trans_ = base_oxts_trans_.inverse();
    init_ = true;
  }

  // get current pose in the initial frame (right-handed reference!)
  double x, y, z;
  GeographicLib::UTMUPS::Forward(gps_msg->latitude, gps_msg->longitude,
    zone, northp, x, y);
  z = gps_msg->altitude;

  tf2::Vector3 t_curr(x, y, z);
  tf2::Quaternion q_curr;
  tf2::fromMsg(imu_msg->orientation, q_curr);
  q_curr.normalize();
  tf2::Transform world_oxts_trans(q_curr, t_curr);

  // This is T_{new_world, base}
  tf2::Transform new_world_base_trans =
    new_world_world_trans_ * world_oxts_trans * oxts_base_trans_;

  // publish shifted gps coordinate in the initial fixed frame
  kitti_msgs::msg::GeoPlanePoint gps_local_msg;
  gps_local_msg.header = gps_msg->header;
  gps_local_msg.header.frame_id = "gps_local";
  gps_local_msg.local_coordinate = tf2::toMsg(new_world_base_trans.getOrigin());
  gps_local_msg.position_covariance = gps_msg->position_covariance;
  gps_pub_->publish(gps_local_msg);

  // publish rotated imu orientation in the inital fixed frame
  sensor_msgs::msg::Imu imu_local_msg = *imu_msg;
  imu_local_msg.header.frame_id = "imu_local";
  // replace orientation to new one based on the new world.
  imu_local_msg.orientation = tf2::toMsg(new_world_base_trans.getRotation());
  // replace linear acc to vehicle acc
  tf2::Vector3 r(base_oxts_trans_.getOrigin());
  tf2::Vector3 omega;
  tf2::fromMsg(imu_msg->angular_velocity, omega);
  tf2::Vector3 linear_acc;
  tf2::fromMsg(imu_msg->linear_acceleration, linear_acc);
  imu_local_msg.linear_acceleration = tf2::toMsg(linear_acc + omega.cross(omega.cross(r)));
  imu_pub_->publish(imu_local_msg);

  // publish oxts tf msg
  geometry_msgs::msg::TransformStamped oxts_tf;
  oxts_tf.header.stamp = gps_msg->header.stamp;
  oxts_tf.header.frame_id = "map";
  oxts_tf.child_frame_id = "oxts_local";
  oxts_tf.transform.translation = tf2::toMsg(new_world_base_trans.getOrigin());
  oxts_tf.transform.rotation = tf2::toMsg(new_world_base_trans.getRotation());

  // Send the transformation
  tf_broadcaster_->sendTransform(oxts_tf);
}

void LocalGpsImu::wait_for_tf()
{
  rclcpp::Time start = rclcpp::Node::now();

  RCLCPP_INFO(
    get_logger(), "Waiting for tf transform data between frames %s and %s to become available",
    "base_link", "oxts_link");

  bool transform_successful = false;

  while (!transform_successful) {
    transform_successful = tf_buffer_->canTransform(
      "base_link", "oxts_link",
      tf2::TimePointZero, tf2::durationFromSec(1.0));

    if (transform_successful) {
      tf2::fromMsg(
        tf_buffer_->lookupTransform("base_link", "oxts_link", tf2::TimePointZero).transform,
        base_oxts_trans_);
      RCLCPP_INFO(get_logger(), "Get the transformation from oxts_link to base_link.");
      break;
    }

    rclcpp::Time now = rclcpp::Node::now();

    if ((now - start).seconds() > 20.0) {
      RCLCPP_WARN_ONCE(
        get_logger(),
        "No transform between frams %s and %s available after %f seconds of waiting. This warning only prints once.",
        "base_link", "velo_link", (now - start).seconds());
    }

    if (!rclcpp::ok()) {
      return;
    }

    rclcpp::WallRate(1.0).sleep();
  }

  rclcpp::Time end = rclcpp::Node::now();
  RCLCPP_INFO(
    get_logger(), "Finished waiting for tf, waited %f seconds", (end - start).seconds());
}

} // namespace local_gps_imu
