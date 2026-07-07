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
      "gps_output_topic", std::string("kitti/vehicle/gps_local"));
    imu_output_topic_ = declare_parameter(
      "imu_output_topic", std::string("kitti/vehicle/imu_local"));
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

    gps_pub_ = create_publisher<kitti_msgs::msg::GeoPlanePoint>(gps_output_topic_, qos);
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

  if (!new_world_world_trans_) {
    // set the first frame as the new world frame.
    double init_x, init_y, init_z;
    GeographicLib::UTMUPS::Forward(
      gps_msg->latitude, gps_msg->longitude, zone, northp, init_x, init_y);
    init_z = gps_msg->altitude;

    tf2::Vector3 t_init(init_x, init_y, init_z);
    tf2::Quaternion q_init;
    tf2::fromMsg(imu_msg->orientation, q_init);
    q_init.normalize();
    tf2::Transform world_oxts_trans(q_init, t_init);

    // T_{new_world, world}
    tf2::Transform new_world_world_trans;
    new_world_world_trans.mult(base_oxts_trans_, world_oxts_trans.inverse());
    new_world_world_trans_ = new_world_world_trans;
  }

  // get current pose in the initial frame (right-handed reference!)
  double x, y, z;
  GeographicLib::UTMUPS::Forward(
    gps_msg->latitude, gps_msg->longitude, zone, northp, x, y);
  z = gps_msg->altitude;

  tf2::Vector3 t_curr(x, y, z);
  tf2::Quaternion q_curr;
  tf2::fromMsg(imu_msg->orientation, q_curr);
  q_curr.normalize();
  tf2::Transform world_oxts_trans(q_curr, t_curr);

  // This is T_{new_world, base}
  tf2::Transform new_world_base_trans =
    *new_world_world_trans_ * world_oxts_trans * oxts_base_trans_;

  // publish shifted gps coordinate in the initial fixed frame
  kitti_msgs::msg::GeoPlanePoint gps_local_msg;
  gps_local_msg.header = gps_msg->header;
  gps_local_msg.header.frame_id = base_frame_id_;
  gps_local_msg.local_coordinate = tf2::toMsg(new_world_base_trans.getOrigin());
  gps_local_msg.position_covariance = gps_msg->position_covariance;
  gps_pub_->publish(gps_local_msg);

  // publish rotated imu orientation in the inital fixed frame
  sensor_msgs::msg::Imu imu_local_msg = *imu_msg;
  imu_local_msg.header.frame_id = base_frame_id_;
  // replace orientation to new one based on the new world.
  imu_local_msg.orientation = tf2::toMsg(new_world_base_trans.getRotation());
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
  imu_local_msg.linear_acceleration =
    tf2::toMsg(linear_acc + 2 * omega.cross(nu) + omega.cross(omega.cross(r)));
  imu_pub_->publish(imu_local_msg);

  // publish velocity in local
  geometry_msgs::msg::TwistStamped vel_local_msg = *vel_msg;
  vel_local_msg.header.frame_id = base_frame_id_;
  // nu_{car} = R_{car,imu}(nu_{imu}+omega^*r)
  vel_local_msg.twist.linear = tf2::toMsg(nu + omega.cross(r));
  vel_pub_->publish(vel_local_msg);

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
