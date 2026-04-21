#include "pillar_detector_pkg/pillar_detector_node_tf.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pillar_detector_pkg::PillarDetectorTFNode>());
  rclcpp::shutdown();
  return 0;
}
