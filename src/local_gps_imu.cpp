// ROS header
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// geographicLib header
#include <GeographicLib/UTMUPS.hpp>

// local header
#include "local_gps_imu/local_gps_imu.hpp"


namespace local_gps_imu
{

LocalGpsImu::LocalGpsImu()
: Node("local_gps_imu_node")
{
  // Initialize ROS2 parameters with validation
  if (!initialize_parameters()) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize parameters");
    rclcpp::shutdown();
    return;
  }

  // Initialize ROS2 publishers, subscribers, TF, and message filters
  if (!initialize_ros_components()) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize ROS components");
    rclcpp::shutdown();
    return;
  }

  wait_for_tf();

  RCLCPP_INFO(get_logger(), "Local GPS/IMU node initialized successfully");
}

bool LocalGpsImu::initialize_parameters()
{
  try {
    // Input topics
    imu_input_topic_ = declare_parameter("imu_input_topic", std::string("kitti/oxts/imu"));
    gps_input_topic_ = declare_parameter("gps_input_topic", std::string("kitti/oxts/gps/fix"));
    vel_input_topic_ = declare_parameter("vel_input_topic", std::string("kitti/oxts/gps/vel"));

    // Output topics
    gps_output_topic_ = declare_parameter(
      "gps_output_topic", std::string("kitti/vehicle/gps"));
    imu_output_topic_ = declare_parameter(
      "imu_output_topic", std::string("kitti/vehicle/imu"));
    vel_output_topic_ = declare_parameter(
      "vel_output_topic", std::string("kitti/vehicle/velocity"));

    // Queue sizes
    queue_size_ = declare_parameter<int>("queue_size", 10);
    sync_queue_size_ = declare_parameter<int>("sync_queue_size", 10);

    if (queue_size_ <= 0 || sync_queue_size_ <= 0) {
      RCLCPP_ERROR(
        get_logger(), "Invalid queue sizes: queue_size=%d, sync_queue_size=%d",
        queue_size_, sync_queue_size_);
      return false;
    }

    // TF wait timeout (seconds). Waiting continues past this, it just starts warning.
    tf_wait_timeout_ = declare_parameter<double>("tf_wait_timeout", 20.0);
    if (tf_wait_timeout_ <= 0.0) {
      RCLCPP_ERROR(get_logger(), "Invalid tf_wait_timeout: %.2f", tf_wait_timeout_);
      return false;
    }

    RCLCPP_INFO(get_logger(), "Parameters initialized successfully");
    RCLCPP_INFO(
      get_logger(), "Input topics: %s, %s, %s",
      imu_input_topic_.c_str(), gps_input_topic_.c_str(), vel_input_topic_.c_str());
    RCLCPP_INFO(
      get_logger(), "Output topics: %s, %s, %s",
      gps_output_topic_.c_str(), imu_output_topic_.c_str(), vel_output_topic_.c_str());

    return true;

  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception during parameter initialization: %s", e.what());
    return false;
  }
}

bool LocalGpsImu::initialize_ros_components()
{
  try {
    rclcpp::QoS qos(queue_size_);
    // GPS/IMU/velocity are low-rate, safety-relevant signals -- explicitly reliable,
    // unlike image and lidar QoS which are deliberately best-effort.
    qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    qos.durability(rclcpp::DurabilityPolicy::Volatile);
    qos.history(rclcpp::HistoryPolicy::KeepLast);

    // Sync GPS and IMU msg. NOTE: message_filters::Subscriber::subscribe() takes an
    // rclcpp::QoS directly as of Jazzy/Lyrical -- the old Humble-era pattern of
    // qos.get_rmw_qos_profile() no longer matches any overload and fails to build.
    imu_sub_.subscribe(this, imu_input_topic_, qos);
    gps_sub_.subscribe(this, gps_input_topic_, qos);
    vel_sub_.subscribe(this, vel_input_topic_, qos);

    sync_ = std::make_shared<message_filters::Synchronizer<policy_t>>(
      policy_t(sync_queue_size_), imu_sub_, gps_sub_, vel_sub_);
    sync_->registerCallback(&LocalGpsImu::sync_callback, this);

    gps_pub_ = create_publisher<av_msgs::msg::GeoPlanePoint>(gps_output_topic_, qos);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_output_topic_, qos);
    vel_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(vel_output_topic_, qos);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(get_logger(), "ROS components initialized successfully");
    return true;

  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Exception during ROS component initialization: %s", e.what());
    return false;
  }
}

void LocalGpsImu::sync_callback(
  const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg,
  const sensor_msgs::msg::NavSatFix::ConstSharedPtr gps_msg,
  const geometry_msgs::msg::TwistStamped::ConstSharedPtr vel_msg)
{
  // dummy variables
  int zone;
  bool northp;

  // get current pose in the world (UTM/map) frame
  double x, y, z;
  GeographicLib::UTMUPS::Forward(
    gps_msg->latitude, gps_msg->longitude, zone, northp, x, y);
  z = gps_msg->altitude;

  if (!origin_captured_) {
    origin_x_ = x;
    origin_y_ = y;
    origin_z_ = z;
    origin_captured_ = true;
  }

  tf2::Vector3 t_curr(x, y, z);
  tf2::Quaternion q_curr;
  tf2::fromMsg(imu_msg->orientation, q_curr);
  q_curr.normalize();
  tf2::Transform world_oxts_trans(q_curr, t_curr);

  // This is T_{world, base} -- world is the raw UTM/map frame (no origin shift)
  tf2::Transform new_world_base_trans = world_oxts_trans * oxts_base_trans_;

  // publish gps coordinate of base_link in the map (UTM) frame -- raw UTM,
  // this is what ekf_localizer consumes and needs the true value for.
  av_msgs::msg::GeoPlanePoint gps_out_msg;
  gps_out_msg.header = gps_msg->header;
  gps_out_msg.header.frame_id = base_frame_id_;
  gps_out_msg.position = tf2::toMsg(new_world_base_trans.getOrigin());
  gps_out_msg.position_covariance = gps_msg->position_covariance;
  gps_pub_->publish(gps_out_msg);

  // publish rotated imu orientation in the map (UTM) frame
  sensor_msgs::msg::Imu imu_out_msg = *imu_msg;
  imu_out_msg.header.frame_id = base_frame_id_;
  // replace orientation to new one based on the world frame.
  imu_out_msg.orientation = tf2::toMsg(new_world_base_trans.getRotation());
  // replace linear acc to vehicle acc
  tf2::Vector3 r(base_oxts_trans_.getOrigin());
  tf2::Vector3 omega;
  tf2::fromMsg(imu_msg->angular_velocity, omega);
  tf2::Vector3 nu;
  tf2::fromMsg(vel_msg->twist.linear, nu);
  tf2::Vector3 linear_acc;
  tf2::fromMsg(imu_msg->linear_acceleration, linear_acc);
  // The equation is
  // alpha_car = R_{car,imu}(alpha_{imu}+2 w^*nu_{imu} + \dot{w}^*r + w^w^*r). R = I in this case
  // \dot{w} is angular acc, which we don't have it. So, simply ignore this value
  imu_out_msg.linear_acceleration =
    tf2::toMsg(linear_acc + 2 * omega.cross(nu) + omega.cross(omega.cross(r)));
  imu_pub_->publish(imu_out_msg);

  // publish velocity in local
  geometry_msgs::msg::TwistStamped vel_out_msg = *vel_msg;
  vel_out_msg.header.frame_id = base_frame_id_;
  // nu_{car} = R_{car,imu}(nu_{imu}+omega^*r)
  vel_out_msg.twist.linear = tf2::toMsg(nu + omega.cross(r));
  vel_pub_->publish(vel_out_msg);

  // publish oxts tf msg -- offset by the first-fix origin so this stays
  // small-magnitude. Large per-frame TF values render as visible jitter in
  // rviz even when the composed transform back to a "small" parent frame
  // would be exact in double precision, since rviz builds one Ogre scene
  // node per TF frame and casts each independently to float32.
  geometry_msgs::msg::TransformStamped oxts_tf;
  oxts_tf.header.stamp = gps_msg->header.stamp;
  oxts_tf.header.frame_id = "map";
  oxts_tf.child_frame_id = "oxts_local";
  oxts_tf.transform.translation.x = new_world_base_trans.getOrigin().x() - origin_x_;
  oxts_tf.transform.translation.y = new_world_base_trans.getOrigin().y() - origin_y_;
  oxts_tf.transform.translation.z = new_world_base_trans.getOrigin().z() - origin_z_;
  oxts_tf.transform.rotation = tf2::toMsg(new_world_base_trans.getRotation());

  // Send the transformation
  tf_broadcaster_->sendTransform(oxts_tf);
}

void LocalGpsImu::wait_for_tf()
{
  rclcpp::Time start = rclcpp::Node::now();

  RCLCPP_INFO(
    get_logger(), "Waiting for tf transform data between frames %s and %s to become available",
    base_frame_id_.c_str(), oxts_frame_id_.c_str());

  bool transform_successful = false;

  while (!transform_successful) {
    transform_successful = tf_buffer_->canTransform(
      base_frame_id_, oxts_frame_id_,
      tf2::TimePointZero, tf2::durationFromSec(1.0));

    if (transform_successful) {
      tf2::fromMsg(
        tf_buffer_->lookupTransform(base_frame_id_, oxts_frame_id_, tf2::TimePointZero).transform,
        base_oxts_trans_);
      oxts_base_trans_ = base_oxts_trans_.inverse();
      RCLCPP_INFO(
        get_logger(), "Got the transformation from %s to %s.",
        oxts_frame_id_.c_str(), base_frame_id_.c_str());
      break;
    }

    rclcpp::Time now = rclcpp::Node::now();

    if ((now - start).seconds() > tf_wait_timeout_) {
      RCLCPP_WARN_ONCE(
        get_logger(),
        "No transform between frames %s and %s available after %.2f seconds of waiting. "
        "This warning only prints once.",
        base_frame_id_.c_str(), oxts_frame_id_.c_str(), (now - start).seconds());
    }

    if (!rclcpp::ok()) {
      return;
    }

    rclcpp::WallRate(1.0).sleep();
  }

  rclcpp::Time end = rclcpp::Node::now();
  RCLCPP_INFO(get_logger(), "Finished waiting for tf, waited %.2f seconds",
    (end - start).seconds());
}

} // namespace local_gps_imu
