#include "data_comm_analysis/data_comm_analysis.hpp"

#include <utility>

namespace data_comm_analysis
{

DataCommAnalysis::DataCommAnalysis(rclcpp::Node::SharedPtr node)
: node_(std::move(node))
{
  altitude_pub_ = node_->create_publisher<std_msgs::msg::Int32>("/alt_bar", 10);
  raw_b1_payload_pub_ = node_->create_publisher<std_msgs::msg::UInt8MultiArray>("/alt_bar_raw_payload", 10);

  registerFrameHandler(
    BAROMETRIC_ALTITUDE_FRAME_ID,
    [this](const std::vector<uint8_t> & data) {
      handleBarometricAltitudeFrame(data);
    });

  RCLCPP_INFO(node_->get_logger(), "DataCommAnalysis created");
}

DataCommAnalysis::~DataCommAnalysis()
{
  if (serial_comm_) {
    serial_comm_->stop_protocol_receive();
    serial_comm_->close();
  }
}

bool DataCommAnalysis::initialize(const std::string & port_name, uint32_t baud_rate)
{
  serial_comm_ = std::make_unique<serial_comm::SerialComm>();
  if (!serial_comm_->initialize(port_name, baud_rate)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "Failed to initialize serial port %s at %u baud: %s",
      port_name.c_str(), baud_rate, serial_comm_->get_last_error().c_str());
    return false;
  }

  serial_comm_->start_protocol_receive(
    [this](uint8_t id, const std::vector<uint8_t> & data) {
      protocolDataHandler(id, data);
    },
    [this](const std::string & err) {
      RCLCPP_WARN(node_->get_logger(), "Serial protocol error: %s", err.c_str());
    });

  RCLCPP_INFO(
    node_->get_logger(),
    "Serial receive started on %s at %u baud. Waiting for frame [AA FF B1 04 ... sc1 sc2]",
    port_name.c_str(), baud_rate);
  return true;
}

void DataCommAnalysis::registerFrameHandler(uint8_t id, FrameHandler handler)
{
  frame_handlers_[id] = std::move(handler);
}

bool DataCommAnalysis::sendCustomFrame(uint8_t id, const std::vector<uint8_t> & payload)
{
  if (!serial_comm_ || !serial_comm_->is_open()) {
    RCLCPP_WARN(node_->get_logger(), "Serial is not ready, cannot send frame 0x%02X", id);
    return false;
  }

  const bool ok = serial_comm_->send_protocol_data(id, static_cast<uint8_t>(payload.size()), payload);
  if (!ok) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Failed to send frame 0x%02X: %s",
      id, serial_comm_->get_last_error().c_str());
  }
  return ok;
}

void DataCommAnalysis::protocolDataHandler(uint8_t id, const std::vector<uint8_t> & data)
{
  const auto it = frame_handlers_.find(id);
  if (it == frame_handlers_.end()) {
    RCLCPP_DEBUG_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 3000,
      "Unhandled frame id=0x%02X, len=%zu", id, data.size());
    return;
  }

  it->second(data);
}

void DataCommAnalysis::handleBarometricAltitudeFrame(const std::vector<uint8_t> & data)
{
  if (data.size() != BAROMETRIC_ALTITUDE_LEN) {
    RCLCPP_WARN(
      node_->get_logger(),
      "Invalid 0xB1 payload length: expect %u, got %zu",
      static_cast<unsigned>(BAROMETRIC_ALTITUDE_LEN), data.size());
    return;
  }

  const int32_t alt_bar = static_cast<int32_t>(
    (static_cast<uint32_t>(data[0])) |
    (static_cast<uint32_t>(data[1]) << 8) |
    (static_cast<uint32_t>(data[2]) << 16) |
    (static_cast<uint32_t>(data[3]) << 24));

  std_msgs::msg::Int32 msg;
  msg.data = alt_bar;
  altitude_pub_->publish(msg);

  std_msgs::msg::UInt8MultiArray raw_msg;
  raw_msg.data = data;
  raw_b1_payload_pub_->publish(raw_msg);

  RCLCPP_DEBUG_THROTTLE(
    node_->get_logger(), *node_->get_clock(), 5000,
    "RX 0xB1 alt_bar=%d", alt_bar);
}

}  // namespace data_comm_analysis
