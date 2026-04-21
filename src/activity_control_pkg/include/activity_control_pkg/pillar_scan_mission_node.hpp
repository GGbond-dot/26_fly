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

namespace activity_control_pkg
{

struct ScanWaypoint
{
  double x_cm;
  double y_cm;
  double z_cm;
  double yaw_deg;
};

// 柱子扫描任务节点：硬编码三航点
//   0) (0,0,40,0)     起飞悬停
//   1) (250,0,40,0)   沿左边直飞；到达 0 时发 enable=true；到达 1 时发 enable=false
//   2) (250,0,4,0)    降落
// 不依赖原 route_target_publisher，直接发 /target_position。
class PillarScanMissionNode : public rclcpp::Node
{
public:
  explicit PillarScanMissionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void heightCallback(const std_msgs::msg::Int16::SharedPtr msg);
  void monitorTimerCallback();

  bool getCurrentPose(double & x_cm, double & y_cm, double & yaw_deg);
  bool isReached(const ScanWaypoint & wp, double x_cm, double y_cm,
                 double z_cm, double yaw_deg) const;

  void publishTarget(const ScanWaypoint & wp);
  void publishEnable(bool on);
  void advance();

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

  // ── 航点 ──
  std::vector<ScanWaypoint> waypoints_;

  // ── ROS ──
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr target_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr               enable_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr              active_controller_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr              mission_complete_pub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr           height_sub_;
  rclcpp::TimerBase::SharedPtr                                    monitor_timer_;

  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ── 状态 ──
  mutable std::mutex mutex_;
  std::size_t current_idx_;
  bool has_height_;
  double current_height_cm_;
  bool mission_complete_sent_;
};

}  // namespace activity_control_pkg
