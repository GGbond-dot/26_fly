#include <rclcpp/rclcpp.hpp>

#include "fire_control_pkg/fire_mission_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<fire_control_pkg::FireMissionNode>());
  rclcpp::shutdown();
  return 0;
}
