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
#include <std_msgs/msg/u_int8.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace spray_control_pkg
{

// 一个航点。spray=true 表示到点悬停后开激光笔“撒药”，
// 否则只是过渡航点（飞越非播撒区 / 返航）。
struct SprayWaypoint
{
  double x_cm;
  double y_cm;
  double z_cm;
  double yaw_deg;
  bool   spray;        // 到点后是否点亮激光笔
  int    block_id;     // 区块编号 1~28（过渡航点用 0）
  const char * tag;    // 日志标签
};

enum class MissionPhase
{
  TAKEOFF,     // “十”字起降点垂直起飞，升至巡航高度 150cm
  GOTO_A,      // 飞往播撒起点区块 A（编号 21）
  COVERAGE,    // 按蛇形全覆盖路径逐格飞行 + 撒药
  RETURN,      // 飞回起降点上空
  LAND,        // 垂直降落，几何中心对准“十”字（±10cm）
  DONE
};

// 植保飞行器（2021 电赛 G 题）撒药任务状态机。
//
// 与飞控/执行机构的接口（沿用 activity_control_pkg 既有约定）：
//   发布 /target_position   Float32MultiArray = [x_cm, y_cm, z_cm, yaw_deg]
//   发布 /active_controller UInt8  (2=位置控制器接管, 0=停)
//   发布 /laser_control     UInt8  (1=激光笔开, 0=关)   ← 新执行机构，需 STM32 固件支持
//   发布 /mission_complete  Empty
//   订阅 高度反馈           Int16  (cm)
//   位姿通过 tf2: map_frame -> base_frame
class SprayMissionNode : public rclcpp::Node
{
public:
  explicit SprayMissionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void heightCallback(const std_msgs::msg::Int16::SharedPtr msg);
  void monitorTimerCallback();

  bool getCurrentPose(double & x_cm, double & y_cm, double & yaw_deg);
  bool isReached(const SprayWaypoint & wp, double x_cm, double y_cm,
                 double z_cm, double yaw_deg) const;

  void publishTarget(const SprayWaypoint & wp);
  void publishLaser(bool on);
  void advance();

  // 撒药动作：到达 spray 航点后，按 blink 次数闪烁激光笔。
  // 返回 true 表示本航点撒药已完成。
  bool runSpraySequence();

  // 生成全覆盖蛇形航点表。grid 由 green_blocks_ 参数提供（见 .cpp 的 TODO）。
  void buildCoverageWaypoints();

  static double meterToCm(double v) { return v * 100.0; }
  double normalizeAngleDeg(double angle_deg) const;

  // ── 参数 ──
  std::string map_frame_;
  std::string base_frame_;

  double pos_tol_cm_;
  double yaw_tol_deg_;
  double height_tol_cm_;

  double flight_height_cm_;   // 巡航高度，默认 150
  double land_height_cm_;
  double home_x_cm_;          // 起降点“十”字坐标
  double home_y_cm_;

  int    spray_blink_count_;  // 每格闪烁次数（1~3，规则要求）
  double spray_on_sec_;       // 单次点亮时长（0.5*周期）
  double spray_off_sec_;

  // ── 航点 / 状态机 ──
  std::vector<SprayWaypoint> waypoints_;
  MissionPhase phase_;
  std::size_t  current_idx_;

  // 撒药子状态
  bool        spraying_;
  int         blink_done_;
  bool        laser_on_;
  rclcpp::Time blink_edge_time_;

  // ── ROS ──
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr target_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr             active_controller_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr             laser_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr             mission_complete_pub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr          height_sub_;
  rclcpp::TimerBase::SharedPtr                                   monitor_timer_;

  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  mutable std::mutex mutex_;
  bool   has_height_;
  double current_height_cm_;
  bool   mission_complete_sent_;
};

}  // namespace spray_control_pkg
