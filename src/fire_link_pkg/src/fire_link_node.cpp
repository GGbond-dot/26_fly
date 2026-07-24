// fire_link_node — 无人机侧机-车 UDP 桥（2023 电赛 G 题）
//
// 两块板 ROS_DOMAIN_ID 不同（车 10 / 飞机 26），DDS 跨机不互通，故用定长 UDP 包单播。
// 包格式见 fire_link_pkg/fire_link_packet.hpp。
//
// 组网：两机接同一台路由器（车板自建热点实测效果差，已弃用），两端均为固定 IP。
//       无人机自己做热点：机 = 10.42.0.1（网关），车 = 10.42.0.163。
//       本节点只需知道车的地址，不需要知道自己的。
//       车端仍保留三级地址解析（学发送方地址→静态参数→子网广播）作为安全网，
//       因为 sendto 发到错误地址是不报错的，现场极难排查。
//       后续计划改蓝牙——届时只需替换本节点的收发实现，
//       /fire_start、/car_status 这些话题接口不变，下游一行不用动。
//
// 机 → 车：
//   遥测      32B 0xF14E type=1 → 车 :8892  定频（默认 1Hz，题目要求每秒 1 次）
//   火源上报  16B 0xFC11        → 车 :8889  收到 /fire_report 即连发，车端零改动
// 车 → 机：
//   启动包    32B 0xF14E type=3 ← 本机 :8893  → 发布 /fire_start
//   任务状态  裸 ASCII 字符串    ← 本机 :8890  → 发布 /car_status

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/string.hpp"

#include "fire_link_pkg/fire_link_packet.hpp"

using fire_link::FireEventPacket;
using fire_link::FireLinkPacket;

class FireLinkNode : public rclcpp::Node
{
public:
  FireLinkNode()
  : Node("fire_link_node")
  {
    // 消防车在路由器网段下的固定地址。无人机自己的地址由 DHCP 分配，本节点不关心。
    car_ip_             = declare_parameter<std::string>("car_ip", "10.42.0.163");
    telemetry_port_     = declare_parameter<int>("telemetry_port", 8892);
    fire_report_port_   = declare_parameter<int>("fire_report_port", 8889);
    start_listen_port_  = declare_parameter<int>("start_listen_port", 8893);
    status_listen_port_ = declare_parameter<int>("status_listen_port", 8890);
    // 题目要求"每秒 1 次"，故默认 1Hz。车端显示器想更跟手可以调高，多发不违规。
    telemetry_rate_hz_  = declare_parameter<double>("telemetry_rate_hz", 1.0);
    report_repeat_      = declare_parameter<int>("report_repeat", 5);

    setupTxSocket();
    start_fd_  = bindSocket(start_listen_port_, "start");
    status_fd_ = bindSocket(status_listen_port_, "status");

    fire_start_pub_ = create_publisher<std_msgs::msg::Empty>(
      "/fire_start", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    car_status_pub_ = create_publisher<std_msgs::msg::String>("/car_status", rclcpp::QoS(10));

    drone_pose_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/drone_pose", rclcpp::QoS(10),
      [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 2) return;
        x_dm_ = msg->data[0];
        y_dm_ = msg->data[1];
        has_pose_ = true;
      });

    patrol_distance_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/patrol_distance", rclcpp::QoS(10),
      [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.empty()) return;
        distance_dm_ = msg->data[0];
      });

    height_sub_ = create_subscription<std_msgs::msg::Int16>(
      "/height", rclcpp::QoS(10),
      [this](const std_msgs::msg::Int16::SharedPtr msg) {
        height_dm_ = static_cast<float>(msg->data) / 10.0f;
      });

    // 从 /fire_status 的文本里取阶段名，转成包里的 1 字节枚举。
    // 状态机不额外开一个话题专供本节点，避免两处状态定义漂移。
    fire_status_sub_ = create_subscription<std_msgs::msg::String>(
      "/fire_status", rclcpp::QoS(10),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        phase_ = parsePhase(msg->data);
      });

    fire_report_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "/fire_report", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 2) return;
        sendFireReport(msg->data[0], msg->data[1]);
      });

    const double period = 1.0 / std::max(telemetry_rate_hz_, 0.1);
    telemetry_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period)),
      std::bind(&FireLinkNode::sendTelemetry, this));

    // 50Hz 轮询两个接收口（启动包是连发的，早一帧收到早一帧起飞）
    rx_timer_ = create_wall_timer(
      std::chrono::milliseconds(20), std::bind(&FireLinkNode::pollSockets, this));

    RCLCPP_INFO(get_logger(),
      "fire_link: 车=%s  遥测→:%d @%.1fHz  火源→:%d  启动←:%d  状态←:%d",
      car_ip_.c_str(), telemetry_port_, telemetry_rate_hz_,
      fire_report_port_, start_listen_port_, status_listen_port_);
  }

  ~FireLinkNode() override
  {
    if (tx_fd_ >= 0) ::close(tx_fd_);
    if (start_fd_ >= 0) ::close(start_fd_);
    if (status_fd_ >= 0) ::close(status_fd_);
  }

private:
  void setupTxSocket()
  {
    tx_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (tx_fd_ < 0) throw std::runtime_error("fire_link: failed to create TX socket");

    in_addr car_addr{};
    if (::inet_pton(AF_INET, car_ip_.c_str(), &car_addr) != 1) {
      ::close(tx_fd_);
      throw std::runtime_error("fire_link: invalid car_ip: " + car_ip_);
    }
    std::memset(&telemetry_addr_, 0, sizeof(telemetry_addr_));
    telemetry_addr_.sin_family = AF_INET;
    telemetry_addr_.sin_addr = car_addr;
    telemetry_addr_.sin_port = htons(static_cast<uint16_t>(telemetry_port_));

    fire_addr_ = telemetry_addr_;
    fire_addr_.sin_port = htons(static_cast<uint16_t>(fire_report_port_));
  }

  int bindSocket(int port, const char * what)
  {
    const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
      throw std::runtime_error(std::string("fire_link: failed to create ") + what + " socket");
    }
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(static_cast<uint16_t>(port));
    if (::bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
      ::close(fd);
      throw std::runtime_error(
        std::string("fire_link: failed to bind ") + what + " UDP port " + std::to_string(port));
    }
    const int flags = ::fcntl(fd, F_GETFL, 0);
    ::fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    return fd;
  }

  static uint32_t nowMs()
  {
    return static_cast<uint32_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
  }

  void sendTo(const void * data, std::size_t len, const sockaddr_in & dest, const char * what)
  {
    const ssize_t sent = ::sendto(tx_fd_, data, len, 0,
      reinterpret_cast<const sockaddr *>(&dest), sizeof(dest));
    if (sent != static_cast<ssize_t>(len)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "fire_link: 发送 %s 失败: %s", what, std::strerror(errno));
    }
  }

  void sendTelemetry()
  {
    if (!has_pose_) return;   // 位姿还没出来（tf 未就绪）就先不发，免得车端显示 (0,0)

    FireLinkPacket pkt{};
    pkt.magic       = fire_link::kMagic;
    pkt.type        = fire_link::TYPE_TELEMETRY;
    pkt.phase       = phase_;
    pkt.seq         = telemetry_seq_++;
    pkt.stamp_ms    = nowMs();
    pkt.x_dm        = x_dm_;
    pkt.y_dm        = y_dm_;
    pkt.distance_dm = distance_dm_;
    pkt.height_dm   = height_dm_;
    sendTo(&pkt, sizeof(pkt), telemetry_addr_, "遥测");
  }

  // 火源坐标走车端已有的 fire_event_bridge（16B / 0xFC11 / :8889），车端零改动。
  // 这是一次性事件，丢了就没有第二次机会 → 同一个 seq 连发 report_repeat 次。
  // 车端按 seq 的 int16 差值判新旧，重复包不会触发多次出发。
  void sendFireReport(float x_dm, float y_dm)
  {
    FireEventPacket pkt{};
    pkt.magic    = fire_link::kFireEventMagic;
    pkt.seq      = report_seq_++;
    pkt.x_dm     = x_dm;
    pkt.y_dm     = y_dm;
    pkt.reserved = 0.0f;
    for (int i = 0; i < report_repeat_; ++i) {
      sendTo(&pkt, sizeof(pkt), fire_addr_, "火源上报");
    }
    RCLCPP_INFO(get_logger(), "已上报火源坐标 (%.1f, %.1f) dm → 消防车（seq=%u，连发 %d 次）",
      x_dm, y_dm, static_cast<unsigned>(pkt.seq), report_repeat_);
  }

  void pollSockets()
  {
    pollStart();
    pollCarStatus();
  }

  void pollStart()
  {
    // 多留一个字节：超长数据报若被静默截断到 sizeof(pkt)，会被误当成合法包。
    std::array<uint8_t, sizeof(FireLinkPacket) + 1> buf{};
    while (true) {
      const ssize_t n = ::recv(start_fd_, buf.data(), buf.size(), 0);
      if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) break;   // 读空
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "fire_link: 启动口 recv 失败: %s", std::strerror(errno));
        break;
      }
      if (n != static_cast<ssize_t>(sizeof(FireLinkPacket))) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "fire_link: 丢弃长度异常的启动包（%zd 字节）", n);
        continue;
      }
      FireLinkPacket pkt{};
      std::memcpy(&pkt, buf.data(), sizeof(pkt));
      if (pkt.magic != fire_link::kMagic || pkt.type != fire_link::TYPE_CAR_START) continue;

      // 启动包是连发的，只认第一次。/fire_start 下游本身也做了幂等，这里再挡一层免得日志刷屏。
      if (start_received_) continue;
      start_received_ = true;
      fire_start_pub_->publish(std_msgs::msg::Empty());
      RCLCPP_INFO(get_logger(), "收到消防车启动包（seq=%u）→ 发布 /fire_start",
        static_cast<unsigned>(pkt.seq));
    }
  }

  // 车端 fire_event_bridge 把 /fire_mission_status 的字符串**原样**（无包头、无magic）
  // sendto 到「最近一个发过火源包的 IP」的 8890 口。
  // 所以在无人机上报火源之前，这个口是收不到任何东西的——这是车端的既有行为，不是故障。
  void pollCarStatus()
  {
    std::array<char, 256> buf{};
    while (true) {
      const ssize_t n = ::recv(status_fd_, buf.data(), buf.size(), 0);
      if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) break;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "fire_link: 状态口 recv 失败: %s", std::strerror(errno));
        break;
      }
      if (n == 0) continue;
      const std::string text(buf.data(), static_cast<std::size_t>(n));
      if (text == last_car_status_) continue;   // 车端周期性重发，只在变化时打日志
      last_car_status_ = text;

      std_msgs::msg::String msg;
      msg.data = text;
      car_status_pub_->publish(msg);
      RCLCPP_INFO(get_logger(), "消防车状态：%s", text.c_str());
    }
  }

  // "阶段=巡逻,航点=3/12,…" → 枚举。fire_control_pkg::buildStatusText() 的阶段名与此表对应，
  // 改那边的中文名必须同步改这里。
  static uint8_t parsePhase(const std::string & status)
  {
    struct Entry { const char * name; uint8_t value; };
    // ⚠️ "恢复巡逻" 包含 "巡逻" 作为子串，必须排在 "巡逻" 之前，否则会被误判成 PATROL。
    // 表按"长名/更具体的在前"排列，取第一个命中的。
    static const Entry kTable[] = {
      {"恢复巡逻", fire_link::PHASE_RESUME},
      {"接近火源", fire_link::PHASE_APPROACH},
      {"待命",     fire_link::PHASE_WAIT_START},
      {"起飞",     fire_link::PHASE_TAKEOFF},
      {"巡逻",     fire_link::PHASE_PATROL},
      {"降高",     fire_link::PHASE_DESCEND},
      {"悬停",     fire_link::PHASE_HOVER},
      {"抛包",     fire_link::PHASE_DROP},
      {"返航",     fire_link::PHASE_RETURN},
      {"降落",     fire_link::PHASE_LAND},
      {"完成",     fire_link::PHASE_DONE},
    };
    // 只取 "阶段=" 到第一个逗号之间的那一段。不能拿整条状态串去 find——
    // 后面还有 ",已抛包" 这类字段，会把 "返航,…,已抛包" 误判成 DROP。
    const auto pos = status.find("阶段=");
    if (pos == std::string::npos) return fire_link::PHASE_UNKNOWN;
    const auto begin = pos + std::strlen("阶段=");
    const auto comma = status.find(',', begin);
    const std::string field = status.substr(
      begin, comma == std::string::npos ? std::string::npos : comma - begin);
    for (const auto & e : kTable) {
      if (field.find(e.name) != std::string::npos) return e.value;
    }
    return fire_link::PHASE_UNKNOWN;
  }

  std::string car_ip_;
  int    telemetry_port_{0};
  int    fire_report_port_{0};
  int    start_listen_port_{0};
  int    status_listen_port_{0};
  double telemetry_rate_hz_{1.0};
  int    report_repeat_{5};

  int tx_fd_{-1};
  int start_fd_{-1};
  int status_fd_{-1};
  sockaddr_in telemetry_addr_{};
  sockaddr_in fire_addr_{};

  uint16_t telemetry_seq_{0};
  uint16_t report_seq_{0};
  bool     start_received_{false};
  std::string last_car_status_;

  bool  has_pose_{false};
  float x_dm_{0.0f};
  float y_dm_{0.0f};
  float distance_dm_{0.0f};
  float height_dm_{0.0f};
  uint8_t phase_{fire_link::PHASE_UNKNOWN};

  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr fire_start_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr car_status_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr drone_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr patrol_distance_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr height_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr fire_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr fire_report_sub_;
  rclcpp::TimerBase::SharedPtr telemetry_timer_;
  rclcpp::TimerBase::SharedPtr rx_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FireLinkNode>());
  rclcpp::shutdown();
  return 0;
}
