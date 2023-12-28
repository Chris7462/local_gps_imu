// ROS header
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// local header
#include "ekf_localizer/ekf_localizer.hpp"


namespace ekf_localizer
{

EkfLocalizer::EkfLocalizer()
: Node("ekf_localizer_node"), freq_{40.0}, dt_{1.0/freq_}, gps_init_{false},
  alt_{0.0}, pitch_{0.0}, roll_{0.0}, odom_base_link_trans_(), imu_buff_(),
  gps_buff_(), geo_converter_(), sys_(), imu_model_(), gps_model_(), ekf_()
{
  declare_parameter("init.x", rclcpp::PARAMETER_DOUBLE_ARRAY);
  declare_parameter("init.P", rclcpp::PARAMETER_DOUBLE_ARRAY);

  declare_parameter("eps.x", rclcpp::PARAMETER_DOUBLE);
  declare_parameter("eps.y", rclcpp::PARAMETER_DOUBLE);
  declare_parameter("eps.theta", rclcpp::PARAMETER_DOUBLE);
  declare_parameter("eps.nu", rclcpp::PARAMETER_DOUBLE);
  declare_parameter("eps.omega", rclcpp::PARAMETER_DOUBLE);
  declare_parameter("eps.alpha", rclcpp::PARAMETER_DOUBLE);

  declare_parameter("tau.x", rclcpp::PARAMETER_DOUBLE);
  declare_parameter("tau.y", rclcpp::PARAMETER_DOUBLE);
  declare_parameter("tau.theta", rclcpp::PARAMETER_DOUBLE);
  declare_parameter("tau.omega", rclcpp::PARAMETER_DOUBLE);
  declare_parameter("tau.alpha", rclcpp::PARAMETER_DOUBLE);

  declare_parameter("qchisq.imu", rclcpp::PARAMETER_DOUBLE);
  declare_parameter("qchisq.gps", rclcpp::PARAMETER_DOUBLE);

  rclcpp::QoS qos(10);

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "kitti/imu", qos, std::bind(&EkfLocalizer::imu_callback, this, std::placeholders::_1));

  gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    "kitti/nav_sat_fix", qos, std::bind(&EkfLocalizer::gps_callback, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(dt_ * 1000)),
    std::bind(&EkfLocalizer::run_ekf, this));

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  odom_base_link_trans_.setIdentity();

  // Init system model
  kalman::SystemCov Q = kalman::SystemCov::Zero();
  double eps_x = get_parameter("eps.x").as_double();
  double eps_y = get_parameter("eps.y").as_double();
  double eps_theta = get_parameter("eps.theta").as_double();
  double eps_nu = get_parameter("eps.nu").as_double();
  double eps_omega = get_parameter("eps.omega").as_double();
  double eps_alpha = get_parameter("eps.alpha").as_double();
  Q.diagonal() << eps_x, eps_y, eps_theta, eps_nu, eps_omega, eps_alpha;
  sys_.setCovariance(Q);

  // Init ekf
  std::vector<double> vec_x = get_parameter("init.x").as_double_array();
  kalman::State init_x;
  std::copy(vec_x.begin(), vec_x.end(), init_x.data());

  std::vector<double> vec_P = get_parameter("init.P").as_double_array();
  kalman::StateCov init_P;
  std::copy(vec_P.begin(), vec_P.end(), init_P.data());

  ekf_.init(init_x, init_P);
}

void EkfLocalizer::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mtx_);
  imu_buff_.push(msg);
}

void EkfLocalizer::gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mtx_);
  gps_buff_.push(msg);
}

void EkfLocalizer::run_ekf()
{
  rclcpp::Time current_time = rclcpp::Node::now();

  ekf_.predict(sys_, dt_);
  ekf_.wrapStateYaw();
  auto s = ekf_.getState();

  // Run imu update
  if (!imu_buff_.empty()) {
    mtx_.lock();
    if ((current_time - rclcpp::Time(imu_buff_.front()->header.stamp)).seconds() > 0.1) {  // time sync has problem
      imu_buff_.pop();
      RCLCPP_WARN(this->get_logger(), "Timestamp unaligned, please check your IMU data.");
      mtx_.unlock();
    } else {
      auto msg = imu_buff_.front();
      imu_buff_.pop();
      mtx_.unlock();

      double yaw;
      tf2::getEulerYPR(msg->orientation, yaw, pitch_, roll_);
      double theta = ekf_.limitMeasurementYaw(yaw);
      double omega = msg->angular_velocity.z;
      double alpha = msg->linear_acceleration.x;

      kalman::ImuMeasurement z;
      z << theta, omega, alpha;

      kalman::ImuMeasurementCov R = kalman::ImuMeasurementCov::Zero();
      double tau_theta = get_parameter("tau.theta").as_double();
      double tau_omega = get_parameter("tau.omega").as_double();
      double tau_alpha = get_parameter("tau.alpha").as_double();
      R.diagonal() << tau_theta, tau_omega, tau_alpha;
      imu_model_.setCovariance(R);

      // if the masurement is okay to fuse?
      double qchisq_imu = get_parameter("qchisq.imu").as_double();
      if (ekf_.mahalanobis(imu_model_, z) > qchisq_imu) {
        RCLCPP_INFO(this->get_logger(), "Measurement IMU is over the threshold. Discard this measurement.");
      } else {  // okay to fuse.
        ekf_.update(imu_model_, z);
        ekf_.wrapStateYaw();

        // publish to TF
        tf2::Vector3 t_current(s.x(), s.y(), alt_);
        tf2::Quaternion q_current;
        q_current.setRPY(roll_, pitch_, s.theta());
        q_current.normalize();

        geometry_msgs::msg::TransformStamped odom_base_link_tf;
        odom_base_link_tf.header.stamp = rclcpp::Node::now();
        odom_base_link_tf.header.frame_id = "odom";
        odom_base_link_tf.child_frame_id = "base_link";
        odom_base_link_tf.transform.translation = tf2::toMsg(t_current);
        odom_base_link_tf.transform.rotation = tf2::toMsg(q_current);

        // send the transformation
        tf_broadcaster_->sendTransform(odom_base_link_tf);

        // save the odom->base transform
        tf2::fromMsg(odom_base_link_tf.transform, odom_base_link_trans_);
      }
    }
  }

  // Run gps update
  if (!gps_buff_.empty()) {
    mtx_.lock();
    if ((current_time - rclcpp::Time(gps_buff_.front()->header.stamp)).seconds() > 0.1) {  // time sync has problem
      gps_buff_.pop();
      RCLCPP_WARN(this->get_logger(), "Timestamp unaligned, please check your GPS data.");
      mtx_.unlock();
    } else {
      auto msg = gps_buff_.front();
      gps_buff_.pop();
      mtx_.unlock();

      if (!gps_init_) {
        // use the first gps data as map (0,0,0)
        geo_converter_.Reset(msg->latitude, msg->longitude, msg->altitude);
        gps_init_ = true;
      }

      // gps measurement
      double lat, lon;
      geo_converter_.Forward(msg->latitude, msg->longitude, msg->altitude, lat, lon, alt_);

      kalman::GpsMeasurement z;
      z << lat, lon;

      kalman::GpsMeasurementCov R = kalman::GpsMeasurementCov::Zero();
      // double tau_x = get_parameter("tau.x").as_double();
      // double tau_y = get_parameter("tau.y").as_double();
      R.diagonal() << msg->position_covariance.at(0), msg->position_covariance.at(4);
      gps_model_.setCovariance(R);

      // if the masurement is okay to fuse?
      double qchisq_gps = get_parameter("qchisq.gps").as_double();
      if (ekf_.mahalanobis(gps_model_, z) > qchisq_gps) {
        RCLCPP_INFO(this->get_logger(), "Measurement GPS is over the threshold. Discard this measurement.");
      } else {
        ekf_.update(gps_model_, z);
        ekf_.wrapStateYaw();
      }
    }
  }

  tf2::Vector3 t_current(s.x(), s.y(), alt_);
  tf2::Quaternion q_current;
  q_current.setRPY(roll_, pitch_, s.theta());
  q_current.normalize();

  tf2::Transform map_base_link_trans(q_current, t_current);
  tf2::Transform map_odom_trans;
  map_odom_trans.mult(map_base_link_trans, odom_base_link_trans_.inverse());

  geometry_msgs::msg::TransformStamped map_odom_tf;
  map_odom_tf.header.stamp = rclcpp::Node::now();
  map_odom_tf.header.frame_id = "map";
  map_odom_tf.child_frame_id = "odom";
  map_odom_tf.transform = tf2::toMsg(map_odom_trans);

  // send the transformation
  tf_broadcaster_->sendTransform(map_odom_tf);
}

}  // namespace ekf_localizer
