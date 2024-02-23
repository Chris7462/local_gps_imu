#include "gps_imu_node/gps_shift.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gps_shift_node::GpsShiftNode>());
  rclcpp::shutdown();

  return 0;
}
