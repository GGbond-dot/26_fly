#pragma once

#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace activity_control_pkg
{

struct ScanWaypoint
{
  double x_cm;
  double y_cm;
  double z_cm;
  double yaw_deg;
  double hover_sec;     // 到达后悬停秒数；0 表示立即切下一航点
  const char * tag;     // 日志用标签
};

enum class MissionPhase
{
  SCAN,          // 起飞 + 扫描直线
  WAIT_PILLARS,  // 扫描段到点后，悬停等待 /detected_pillars 结果
  VISIT,         // 依次飞到柱子正上方，每个悬停 hover_sec
  LAND,          // 飞向对角 B 并降落
  DONE
};

// 柱子扫描 + 贪心访问 + 对角降落 任务节点
//   SCAN:         (0,0,fly) → (scan_end_x,0,fly)        扫描期间 /pillar_detect_enable=true
//   WAIT_PILLARS: 在 scan_end 悬停等 /detected_pillars
//   VISIT:        贪心最近邻，逐个飞到每个柱子正上方，悬停 pillar_hover_sec
//   LAND:         飞到 (landing_x, landing_y, fly) → 降到 land_height
class PillarScanMissionNode : public rclcpp::Node
{
public:
  explicit PillarScanMissionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void heightCallback(const std_msgs::msg::Int16::SharedPtr msg);
  void pillarsCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void monitorTimerCallback();

  bool getCurrentPose(double & x_cm, double & y_cm, double & yaw_deg);
  bool isReached(const ScanWaypoint & wp, double x_cm, double y_cm,
                 double z_cm, double yaw_deg) const;

  void publishTarget(const ScanWaypoint & wp);
  void publishEnable(bool on);
  void advance();

  std::vector<std::pair<double, double>> greedyOrder(
    const std::vector<std::pair<double, double>> & pillars,
    double start_x_cm,
    double start_y_cm) const;

  void buildVisitAndLandWaypoints(double cur_x_cm, double cur_y_cm);

  static double meterToCm(double v) { return v * 100.0; }
  static double radToDeg(double v);
  double normalizeAngleDeg(double angle_deg) const;

  // ── 参数 ──
  std::string map_frame_;
  std::string laser_link_frame_;
  std::string target_topic_;
  std::string enable_topic_;

  double pos_tol_cm_;
  double yaw_tol_deg_;
  double height_tol_cm_;

  double flight_height_cm_;
  double land_height_cm_;
  double scan_end_x_cm_;
  double landing_x_cm_;
  double landing_y_cm_;
  double pillar_visit_height_cm_;
  double pillar_hover_sec_;
  double pillar_wait_timeout_sec_;

  // ── 航点 / 状态机 ──
  std::vector<ScanWaypoint> waypoints_;
  MissionPhase phase_;
  std::size_t scan_end_idx_;

  std::vector<std::pair<double, double>> detected_pillars_cm_;
  bool pillars_received_;
  rclcpp::Time wait_pillars_start_time_;

  bool is_hovering_;
  rclcpp::Time hover_start_time_;

  // ── ROS ──
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr    target_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                 enable_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr                active_controller_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr                mission_complete_pub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr             height_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr pillars_sub_;
  rclcpp::TimerBase::SharedPtr                                      monitor_timer_;

  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  mutable std::mutex mutex_;
  std::size_t current_idx_;
  bool has_height_;
  double current_height_cm_;
  bool mission_complete_sent_;
};

}  // namespace activity_control_pkg
