#include "gps_imu_node/imu_rotate.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<imu_rotate_node::ImuRotateNode>());
  rclcpp::shutdown();

  return 0;
}
