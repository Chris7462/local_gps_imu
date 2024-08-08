#include "local_gps_imu/local_gps_imu.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<local_gps_imu::LocalGpsImu>());
  rclcpp::shutdown();

  return 0;
}
