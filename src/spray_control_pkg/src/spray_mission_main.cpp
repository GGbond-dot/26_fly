#include <rclcpp/rclcpp.hpp>

#include "spray_control_pkg/spray_mission_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<spray_control_pkg::SprayMissionNode>());
  rclcpp::shutdown();
  return 0;
}
