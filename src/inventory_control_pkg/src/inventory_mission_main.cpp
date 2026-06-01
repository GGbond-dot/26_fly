#include <rclcpp/rclcpp.hpp>

#include "inventory_control_pkg/inventory_mission_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<inventory_control_pkg::InventoryMissionNode>());
  rclcpp::shutdown();
  return 0;
}
