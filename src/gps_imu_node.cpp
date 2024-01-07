#include "gps_imu_node/gps_imu.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gps_imu_node::GpsImuNode>());
  rclcpp::shutdown();

  return 0;
}
