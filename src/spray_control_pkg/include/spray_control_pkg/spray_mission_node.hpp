#pragma once

#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace spray_control_pkg
{

// 一个航点。
//   spray=true        → 到点等 3 帧 /spray_allowed，见绿才闪激光 2 次（颜色门控撒药）
//   wait_barcode=true → 到点悬停等 /barcode_text，识到后发 /led_digit 显示数字
//   两者均 false      → 普通过渡航点（飞越非播撒区 / 起飞返航）
struct SprayWaypoint
{
  double x_cm;
  double y_cm;
  double z_cm;
  double yaw_deg;
  bool   spray;
  bool   wait_barcode;
  int    block_id;     // 区块编号 1~28（过渡航点用 0）
  const char * tag;
};

enum class MissionPhase
{
  TAKEOFF,     // "十"字起降点垂直起飞，升至巡航高度 150cm
  GOTO_A,      // 飞往播撒起点区块 A（编号 21）
  COVERAGE,    // 蛇形全覆盖；途中可能穿插 BARCODE_OBSERVE 航点
  RETURN,      // 飞回起降点上空
  LAND,        // 垂直降落，几何中心对准"十"字（±10cm）。
               // 若 barcode_number_ > 0 则改为圆周降落（半径 N×10cm）
  DONE
};

// 植保飞行器（2021 电赛 G 题）撒药任务状态机。
//
// 接口约定（与本机搬运代码 + PID/uart 一致；激光复用电磁铁链路）：
//   发布 /target_position        Float32MultiArray = [x_cm, y_cm, z_cm, yaw_deg]
//   发布 /active_controller      UInt8  (2=位置控制器接管, 0=停)
//   发布 /electromagnet_control  UInt8  (1=激光开, 0=关) ← 物理上激光接在电磁铁 GPIO，复用 0x33 帧
//   发布 /led_digit              UInt8  (1~9 → STM32 LED 闪烁次数显示条码数字，新增 0x12 帧)
//   发布 /mission_complete       Empty
//   订阅 /height                 Int16  (cm，由 uart_to_stm32 从 STM32 上报)
//   订阅 /spray_allowed          Bool   (下视相机中心 ROI 见绿色 = true)
//   订阅 /barcode_text           String (pyzbar 识到的 Code128 文本)
//   订阅 /detected_pillar        Float32MultiArray [x_m, y_m] (单杆塔 2D 坐标)
//   位姿通过 tf2: map → laser_link
class SprayMissionNode : public rclcpp::Node
{
public:
  explicit SprayMissionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ── 订阅回调 ──
  void heightCallback(const std_msgs::msg::Int16::SharedPtr msg);
  void sprayAllowedCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void barcodeTextCallback(const std_msgs::msg::String::SharedPtr msg);
  void pillarCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

  void monitorTimerCallback();

  bool getCurrentPose(double & x_cm, double & y_cm, double & yaw_deg);
  bool isReached(const SprayWaypoint & wp, double x_cm, double y_cm,
                 double z_cm, double yaw_deg) const;

  void publishTarget(const SprayWaypoint & wp);
  void publishLaser(bool on);
  void publishLedDigit(uint8_t digit);
  void advance();

  // 撒药子状态机（颜色门控）：到达 spray 航点后调用。
  // 返回 true 表示本航点 spray 流程结束（可能闪了 2 次，也可能跳过）。
  bool runSprayWithColorGate();

  // 条码航点子状态：到达后悬停等 /barcode_text。
  // 返回 true 表示识别完成、已发 LED、可推进下一航点。
  bool runBarcodeObserve();

  // 把单杆塔位置作为观察航点插入到当前航点表（GOTO_A 之后、COVERAGE 之前）。
  void insertBarcodeObserveWaypoint(double pillar_x_m, double pillar_y_m);

  void buildCoverageWaypoints();
  void resetSprayState();
  void resetBarcodeState();

  // 若识到条码数字，返航后改用圆周降落；否则常规降落。
  void prepareFinalLanding();

  static double meterToCm(double v) { return v * 100.0; }
  double normalizeAngleDeg(double angle_deg) const;
  int parseBarcodeNumber(const std::string & text) const;

  // ── 参数 ──
  std::string map_frame_;
  std::string base_frame_;          // 实际是 laser_link（与 PID/uart 一致）

  double pos_tol_cm_;
  double yaw_tol_deg_;
  double height_tol_cm_;

  double flight_height_cm_;          // 巡航高度，默认 150
  double land_height_cm_;
  double home_x_cm_;                 // 起降点"十"字坐标
  double home_y_cm_;

  // 撒药参数
  double spray_decision_timeout_sec_; // 等不到 3 帧颜色就跳过
  double spray_data_stale_timeout_sec_;
  double spray_on_sec_;
  double spray_off_sec_;
  static constexpr int kSprayDecisionFrameCount = 3;

  // 条码任务参数
  bool   enable_barcode_task_;
  double pillar_left_offset_m_;      // 观察点 = (px, py + offset)，对应朋友的 0.8m
  double barcode_target_z_cm_;       // 观察点高度（瞄条码段中心，105~125cm）
  double barcode_wait_timeout_sec_;  // 悬停等条码最长时间

  // 圆周降落参数（发挥 3）
  bool   enable_circle_landing_;
  double circle_radius_digit_div_;   // N×10cm 中的 N 取自哪一位：1=个位（默认）

  // ── 航点 / 阶段 ──
  std::vector<SprayWaypoint> waypoints_;
  MissionPhase phase_;
  std::size_t  current_idx_;

  // 撒药子状态
  bool          spray_active_;
  int           spray_blink_step_;   // -1=未闪, 0=第一次开, 1=灭, 2=第二次开
  rclcpp::Time  spray_start_time_;
  rclcpp::Time  spray_blink_edge_time_;
  rclcpp::Time  last_sampled_allowed_time_;
  int           spray_frame_count_;
  bool          spray_seen_green_;

  bool          laser_on_;

  // 条码子状态
  bool          barcode_observe_active_;
  rclcpp::Time  barcode_observe_start_time_;
  bool          barcode_detected_;
  std::string   latest_barcode_text_;
  int           barcode_number_;     // 解析出的数字（4 位 Code128）

  // 杆塔订阅 / 早期插入
  bool          pillar_received_;
  bool          pillar_inserted_;
  double        pillar_x_m_;
  double        pillar_y_m_;

  // 颜色门控订阅状态
  bool          has_spray_allowed_;
  bool          latest_spray_allowed_;
  rclcpp::Time  last_spray_allowed_stamp_;

  // ── ROS ──
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr target_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr             active_controller_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr             electromagnet_pub_;   // 复用电磁铁链路控激光
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr             led_digit_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr             mission_complete_pub_;

  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr              height_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr               spray_allowed_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr             barcode_text_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr  pillar_sub_;

  rclcpp::TimerBase::SharedPtr  monitor_timer_;

  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  mutable std::mutex mutex_;
  bool   has_height_;
  double current_height_cm_;
  bool   mission_complete_sent_;
  bool   first_publish_done_;
};

}  // namespace spray_control_pkg
