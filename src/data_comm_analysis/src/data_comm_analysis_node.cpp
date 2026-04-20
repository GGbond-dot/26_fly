#include <cstdlib>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "data_comm_analysis/data_comm_analysis.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("data_comm_analysis_node");

  node->declare_parameter<std::string>("serial_port", "/dev/ttyS3");
  node->declare_parameter<int>("baud_rate", 115200);

  const auto serial_port = node->get_parameter("serial_port").as_string();
  const auto baud_rate = static_cast<uint32_t>(node->get_parameter("baud_rate").as_int());

  try {
    auto app = std::make_shared<data_comm_analysis::DataCommAnalysis>(node);
    if (!app->initialize(serial_port, baud_rate)) {
      RCLCPP_ERROR(node->get_logger(), "Failed to initialize DataCommAnalysis");
      rclcpp::shutdown();
      return EXIT_FAILURE;
    }

    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(node->get_logger(), "Exception in main: %s", e.what());
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
