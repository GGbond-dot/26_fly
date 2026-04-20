#ifndef DATA_COMM_ANALYSIS__DATA_COMM_ANALYSIS_HPP_
#define DATA_COMM_ANALYSIS__DATA_COMM_ANALYSIS_HPP_

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <serial_comm/serial_comm.h>

namespace data_comm_analysis
{

class DataCommAnalysis
{
public:
  using FrameHandler = std::function<void(const std::vector<uint8_t> &)>;

  explicit DataCommAnalysis(rclcpp::Node::SharedPtr node);
  ~DataCommAnalysis();

  bool initialize(const std::string & port_name, uint32_t baud_rate);

  void registerFrameHandler(uint8_t id, FrameHandler handler);
  bool sendCustomFrame(uint8_t id, const std::vector<uint8_t> & payload);

private:
  void protocolDataHandler(uint8_t id, const std::vector<uint8_t> & data);
  void handleBarometricAltitudeFrame(const std::vector<uint8_t> & data);

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<serial_comm::SerialComm> serial_comm_;

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr altitude_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr raw_b1_payload_pub_;

  std::unordered_map<uint8_t, FrameHandler> frame_handlers_;

  static constexpr uint8_t BAROMETRIC_ALTITUDE_FRAME_ID = 0xB1;
  static constexpr uint8_t BAROMETRIC_ALTITUDE_LEN = 4;
};

}  // namespace data_comm_analysis

#endif  // DATA_COMM_ANALYSIS__DATA_COMM_ANALYSIS_HPP_
