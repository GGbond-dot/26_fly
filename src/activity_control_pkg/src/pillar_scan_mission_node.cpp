#include "activity_control_pkg/pillar_scan_mission_node.hpp"

#include <angles/angles.h>
#include <chrono>
#include <cmath>
#include <limits>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace activity_control_pkg
{

PillarScanMissionNode::PillarScanMissionNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("pillar_scan_mission", options),
  current_idx_(0),
  has_height_(false),
  current_height_cm_(0.0),
  mission_complete_sent_(false)
{
  map_frame_         = declare_parameter<std::string>("map_frame",         "map");
  laser_link_frame_  = declare_parameter<std::string>("laser_link_frame",  "laser_link");
  target_topic_      = declare_parameter<std::string>("target_topic",      "/target_position");
  enable_topic_      = declare_parameter<std::string>("enable_topic",      "/pillar_detect_enable");

  pos_tol_cm_    = declare_parameter("position_tolerance_cm", 9.0);
  yaw_tol_deg_   = declare_parameter("yaw_tolerance_deg",     5.0);
  height_tol_cm_ = declare_parameter("height_tolerance_cm",   12.0);

  const double flight_z_cm = declare_parameter("flight_height_cm", 40.0);
  const double land_z_cm   = declare_parameter("land_height_cm",    4.0);
  const double scan_end_x_cm = declare_parameter("scan_end_x_cm", 250.0);

  waypoints_ = {
    { 0.0,         0.0, flight_z_cm, 0.0},  // 0: 起飞悬停
    { scan_end_x_cm, 0.0, flight_z_cm, 0.0}, // 1: 扫描终点
    { scan_end_x_cm, 0.0, land_z_cm,   0.0}, // 2: 降落
  };

  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  auto durable_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  target_pub_            = create_publisher<std_msgs::msg::Float32MultiArray>(target_topic_, durable_qos);
  enable_pub_            = create_publisher<std_msgs::msg::Bool>(enable_topic_, durable_qos);
  active_controller_pub_ = create_publisher<std_msgs::msg::UInt8>("/active_controller", durable_qos);
  mission_complete_pub_  = create_publisher<std_msgs::msg::Empty>("/mission_complete", rclcpp::QoS(10).reliable());

  height_sub_ = create_subscription<std_msgs::msg::Int16>(
    "/height", rclcpp::QoS(10),
    std::bind(&PillarScanMissionNode::heightCallback, this, std::placeholders::_1));

  publishEnable(false);            // 初始禁用检测
  publishTarget(waypoints_[0]);    // 先发第 0 航点

  monitor_timer_ = create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&PillarScanMissionNode::monitorTimerCallback, this));

  RCLCPP_INFO(get_logger(),
    "柱子扫描任务启动: 航点 %zu 个, 第一目标 (%.1f, %.1f, %.1f cm)",
    waypoints_.size(), waypoints_[0].x_cm, waypoints_[0].y_cm, waypoints_[0].z_cm);
}

void PillarScanMissionNode::heightCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
  current_height_cm_ = static_cast<double>(msg->data);
  has_height_ = true;
}

bool PillarScanMissionNode::getCurrentPose(double & x_cm, double & y_cm, double & yaw_deg)
{
  try {
    geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(
      map_frame_, laser_link_frame_, tf2::TimePointZero);
    x_cm = meterToCm(tf.transform.translation.x);
    y_cm = meterToCm(tf.transform.translation.y);

    tf2::Quaternion q;
    tf2::fromMsg(tf.transform.rotation, q);
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    yaw_deg = radToDeg(yaw);
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "TF %s <- %s 查询失败: %s",
      map_frame_.c_str(), laser_link_frame_.c_str(), ex.what());
    return false;
  }
}

bool PillarScanMissionNode::isReached(const ScanWaypoint & wp, double x_cm, double y_cm,
                                      double z_cm, double yaw_deg) const
{
  const double dxy  = std::hypot(wp.x_cm - x_cm, wp.y_cm - y_cm);
  const double dz   = wp.z_cm - z_cm;
  const double dyaw = normalizeAngleDeg(wp.yaw_deg - yaw_deg);

  const bool xy_ok  = dxy <= pos_tol_cm_;
  const bool z_ok   = std::fabs(dz) <= height_tol_cm_;
  const bool yaw_ok = std::fabs(dyaw) <= yaw_tol_deg_;

  // 起飞航点只看高度（与原 RouteTargetPublisher 同一语义）
  if (current_idx_ == 0 && wp.z_cm > 20.0) {
    return z_ok;
  }
  // 高度 > 20cm 的中途航点要求 xy+z；降落航点也加上 yaw
  if (wp.z_cm > 20.0) {
    return z_ok && xy_ok;
  }
  return z_ok && xy_ok && yaw_ok;
}

void PillarScanMissionNode::publishTarget(const ScanWaypoint & wp)
{
  std_msgs::msg::Float32MultiArray msg;
  msg.data.resize(4);
  msg.data[0] = static_cast<float>(wp.x_cm);
  msg.data[1] = static_cast<float>(wp.y_cm);
  msg.data[2] = static_cast<float>(wp.z_cm);
  msg.data[3] = static_cast<float>(wp.yaw_deg);
  target_pub_->publish(msg);

  std_msgs::msg::UInt8 active_msg;
  active_msg.data = 2;
  active_controller_pub_->publish(active_msg);

  RCLCPP_INFO(get_logger(),
    "发布航点 %zu: x=%.1f y=%.1f z=%.1f yaw=%.1f",
    current_idx_, wp.x_cm, wp.y_cm, wp.z_cm, wp.yaw_deg);
}

void PillarScanMissionNode::publishEnable(bool on)
{
  std_msgs::msg::Bool msg;
  msg.data = on;
  enable_pub_->publish(msg);
  RCLCPP_INFO(get_logger(), "/pillar_detect_enable = %s", on ? "true" : "false");
}

void PillarScanMissionNode::advance()
{
  ++current_idx_;
  if (current_idx_ < waypoints_.size()) {
    publishTarget(waypoints_[current_idx_]);
  } else {
    if (!mission_complete_sent_) {
      std_msgs::msg::Empty m;
      mission_complete_pub_->publish(m);
      mission_complete_sent_ = true;
    }
    std_msgs::msg::UInt8 stop_msg;
    stop_msg.data = 3;
    active_controller_pub_->publish(stop_msg);
    RCLCPP_INFO(get_logger(), "扫描任务全部航点完成。");
  }
}

void PillarScanMissionNode::monitorTimerCallback()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (current_idx_ >= waypoints_.size()) {
    std_msgs::msg::UInt8 stop_msg;
    stop_msg.data = 3;
    active_controller_pub_->publish(stop_msg);
    return;
  }

  double x_cm = 0.0, y_cm = 0.0, yaw_deg = 0.0;
  if (!getCurrentPose(x_cm, y_cm, yaw_deg)) { return; }
  const double z_cm = has_height_ ? current_height_cm_ : 0.0;

  const auto & wp = waypoints_[current_idx_];

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
    "当前航点 %zu -> (%.1f,%.1f,%.1f,%.1f) 位姿=(%.1f,%.1f,%.1f,%.1f)",
    current_idx_, wp.x_cm, wp.y_cm, wp.z_cm, wp.yaw_deg,
    x_cm, y_cm, z_cm, yaw_deg);

  if (!isReached(wp, x_cm, y_cm, z_cm, yaw_deg)) { return; }

  // ── 到达事件：按航点索引触发 /pillar_detect_enable ──
  if (current_idx_ == 0) {
    // 起飞悬停到位 → 开始检测
    publishEnable(true);
  } else if (current_idx_ == 1) {
    // 飞到扫描终点 → 结束检测
    publishEnable(false);
  }

  RCLCPP_INFO(get_logger(), "航点 %zu 到达。", current_idx_);
  advance();
}

double PillarScanMissionNode::radToDeg(double v)
{
  return v * 180.0 / M_PI;
}

double PillarScanMissionNode::normalizeAngleDeg(double angle_deg) const
{
  const double n = angles::normalize_angle(angles::from_degrees(angle_deg));
  return angles::to_degrees(n);
}

}  // namespace activity_control_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<activity_control_pkg::PillarScanMissionNode>());
  rclcpp::shutdown();
  return 0;
}
