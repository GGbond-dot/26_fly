// VL53 测距节点（维特 VL53-100 串口模式，ASCII 输出）
//
// 用途：雷达 standoff 的备份/交叉校验（笔记 §8，雷达为主、VL53 为备）。
// 传感器普通模式量程仅 ~1.2m、受强光影响，只当雷达盲区时的备份。
//
// 串口输出（默认 115200 / 10Hz，ASCII 逐行）：
//     State;N            N=0 表示测量有效
//     d: NN mm           NN=距离毫米
// 复用 serial_comm 裸流读取（start_async_read）+ 逐行正则解析。
//
// 接线：默认 /dev/ttyS3（串口三，原气压计 data_comm_analysis 链路已停用，让给 VL53）。
//        只要不同时启动 data_comm_analysis 就不会抢口。
//
// 发布：
//     /vl53/distance      Float32  距离（米）
//     /vl53/state         Int32    传感器状态（0=有效）
//     /vl53/valid         Bool     本次读数是否可信（state==0 且在量程内）
//     /vl53/distance_mm   Int32    距离（毫米，调试用）

#include <memory>
#include <mutex>
#include <regex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>

#include "serial_comm/serial_comm.h"

namespace vl53_ranging_pkg
{

class Vl53RangingNode : public rclcpp::Node
{
public:
  Vl53RangingNode()
  : rclcpp::Node("vl53_ranging_node"),
    last_state_(-1),
    has_state_(false)
  {
    serial_port_   = declare_parameter<std::string>("serial_port", "/dev/ttyS3");
    baud_rate_     = static_cast<unsigned int>(declare_parameter<int>("baud_rate", 115200));
    // 普通模式量程约 1.2m，留点余量；超出/为 0 判为无效。
    min_valid_mm_  = declare_parameter<int>("min_valid_mm", 10);
    max_valid_mm_  = declare_parameter<int>("max_valid_mm", 2000);

    distance_pub_    = create_publisher<std_msgs::msg::Float32>("/vl53/distance", 10);
    state_pub_       = create_publisher<std_msgs::msg::Int32>("/vl53/state", 10);
    valid_pub_       = create_publisher<std_msgs::msg::Bool>("/vl53/valid", 10);
    distance_mm_pub_ = create_publisher<std_msgs::msg::Int32>("/vl53/distance_mm", 10);

    serial_ = std::make_unique<serial_comm::SerialComm>();
    if (!serial_->initialize(serial_port_, baud_rate_)) {
      RCLCPP_ERROR(get_logger(), "VL53 串口 %s @ %u 打开失败：%s",
                   serial_port_.c_str(), baud_rate_, serial_->get_last_error().c_str());
      throw std::runtime_error("VL53 serial open failed");
    }

    serial_->start_async_read(
      [this](const std::vector<uint8_t> & bytes) { onBytes(bytes); },
      [this](const std::string & err) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "VL53 串口错误：%s", err.c_str());
      });

    RCLCPP_INFO(get_logger(), "VL53 测距节点启动：%s @ %u，等待 'State;N' + 'd: NN mm'…",
                serial_port_.c_str(), baud_rate_);
  }

  ~Vl53RangingNode() override
  {
    if (serial_) {
      serial_->stop_async_read();
      serial_->close();
    }
  }

private:
  // 裸流回调：累积进行缓冲，按换行切出完整行逐行解析。
  void onBytes(const std::vector<uint8_t> & bytes)
  {
    std::string lines_to_parse;
    {
      std::lock_guard<std::mutex> lk(buf_mutex_);
      line_buf_.append(reinterpret_cast<const char *>(bytes.data()), bytes.size());

      std::size_t nl;
      while ((nl = line_buf_.find('\n')) != std::string::npos) {
        std::string line = line_buf_.substr(0, nl);
        line_buf_.erase(0, nl + 1);
        if (!line.empty() && line.back() == '\r') line.pop_back();
        lines_to_parse += line;
        lines_to_parse += '\n';
      }
      // 防止脏口无换行时缓冲无限增长
      if (line_buf_.size() > 4096) line_buf_.clear();
    }

    std::size_t start = 0, nl;
    while ((nl = lines_to_parse.find('\n', start)) != std::string::npos) {
      parseLine(lines_to_parse.substr(start, nl - start));
      start = nl + 1;
    }
  }

  void parseLine(const std::string & line)
  {
    std::smatch m;
    // State;N
    static const std::regex state_re(R"(State\s*;\s*(-?\d+))", std::regex::icase);
    if (std::regex_search(line, m, state_re)) {
      last_state_ = std::stoi(m[1].str());
      has_state_ = true;
      return;
    }
    // d: NN mm
    static const std::regex dist_re(R"(d\s*:\s*(\d+)\s*mm)", std::regex::icase);
    if (std::regex_search(line, m, dist_re)) {
      const int mm = std::stoi(m[1].str());
      publishReading(mm);
    }
  }

  void publishReading(int mm)
  {
    const int  state = has_state_ ? last_state_ : -1;
    const bool valid = (state == 0) && (mm >= min_valid_mm_) && (mm <= max_valid_mm_);

    std_msgs::msg::Int32 mm_msg;   mm_msg.data = mm;        distance_mm_pub_->publish(mm_msg);
    std_msgs::msg::Int32 st_msg;   st_msg.data = state;     state_pub_->publish(st_msg);
    std_msgs::msg::Bool  vd_msg;   vd_msg.data = valid;     valid_pub_->publish(vd_msg);
    std_msgs::msg::Float32 d_msg;  d_msg.data = mm / 1000.0f; distance_pub_->publish(d_msg);

    RCLCPP_DEBUG(get_logger(), "VL53 d=%d mm state=%d valid=%d", mm, state, valid ? 1 : 0);
  }

  // 参数
  std::string  serial_port_;
  unsigned int baud_rate_;
  int          min_valid_mm_;
  int          max_valid_mm_;

  // 状态
  int  last_state_;
  bool has_state_;

  std::mutex  buf_mutex_;
  std::string line_buf_;

  std::unique_ptr<serial_comm::SerialComm> serial_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr distance_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr   state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr    valid_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr   distance_mm_pub_;
};

}  // namespace vl53_ranging_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<vl53_ranging_pkg::Vl53RangingNode>());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("vl53_ranging_node"), "节点异常退出：%s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
