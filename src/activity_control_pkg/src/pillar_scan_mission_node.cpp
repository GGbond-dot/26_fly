#include "activity_control_pkg/pillar_scan_mission_node.hpp"

#include <algorithm>
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
  phase_(MissionPhase::SCAN),
  scan_end_idx_(1),
  pillars_received_(false),
  is_hovering_(false),
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

  flight_height_cm_        = declare_parameter("flight_height_cm",        40.0);
  land_height_cm_          = declare_parameter("land_height_cm",           4.0);
  scan_end_x_cm_           = declare_parameter("scan_end_x_cm",          250.0);
  landing_x_cm_            = declare_parameter("landing_x_cm",           250.0);
  landing_y_cm_            = declare_parameter("landing_y_cm",          -250.0);
  pillar_visit_height_cm_  = declare_parameter("pillar_visit_height_cm",  150.0);
  pillar_hover_sec_        = declare_parameter("pillar_hover_sec",         1.0);
  pillar_wait_timeout_sec_ = declare_parameter("pillar_wait_timeout_sec",  3.0);

  waypoints_ = {
    ScanWaypoint{ 0.0,            0.0, flight_height_cm_, 0.0, 0.0, "takeoff"},
    ScanWaypoint{ scan_end_x_cm_, 0.0, flight_height_cm_, 0.0, 0.0, "scan_end"},
  };
  scan_end_idx_ = 1;

  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  auto durable_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  target_pub_            = create_publisher<std_msgs::msg::Float32MultiArray>(target_topic_, durable_qos);
  enable_pub_            = create_publisher<std_msgs::msg::Bool>(enable_topic_, durable_qos);
  active_controller_pub_ = create_publisher<std_msgs::msg::UInt8>("/active_controller", durable_qos);
  mission_complete_pub_  = create_publisher<std_msgs::msg::Empty>("/mission_complete", rclcpp::QoS(10).reliable());

  // 高度源：面阵激光 /laser_array/ground_height (Int16, 单位 cm，与飞控 /height 同语义)
  height_sub_ = create_subscription<std_msgs::msg::Int16>(
    "/laser_array/ground_height", rclcpp::QoS(10),
    std::bind(&PillarScanMissionNode::heightCallback, this, std::placeholders::_1));

  // pillar_detector_tf 以 transient_local 发布 /detected_pillars，用相同 QoS 订阅
  auto pillars_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  pillars_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
    "/detected_pillars", pillars_qos,
    std::bind(&PillarScanMissionNode::pillarsCallback, this, std::placeholders::_1));

  publishEnable(false);
  publishTarget(waypoints_[0]);

  monitor_timer_ = create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&PillarScanMissionNode::monitorTimerCallback, this));

  RCLCPP_INFO(get_logger(),
    "任务启动: 起飞(0,0,%.0f) → 扫描终点(%.0f,0) → 访问柱子(悬停%.1fs) → 降落(%.0f,%.0f)",
    flight_height_cm_, scan_end_x_cm_, pillar_hover_sec_,
    landing_x_cm_, landing_y_cm_);
}

void PillarScanMissionNode::heightCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
  // /laser_array/ground_height 单位 cm（与飞控 /height 一致）
  current_height_cm_ = static_cast<double>(msg->data);
  has_height_ = true;
}

void PillarScanMissionNode::pillarsCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  detected_pillars_cm_.clear();
  if (msg->data.size() % 2 != 0) {
    RCLCPP_WARN(get_logger(), "/detected_pillars 数据长度异常: %zu", msg->data.size());
    pillars_received_ = true;
    return;
  }
  for (std::size_t k = 0; k + 1 < msg->data.size(); k += 2) {
    const double xm = static_cast<double>(msg->data[k]);
    const double ym = static_cast<double>(msg->data[k + 1]);
    detected_pillars_cm_.emplace_back(xm * 100.0, ym * 100.0);
  }
  pillars_received_ = true;
  RCLCPP_INFO(get_logger(), "收到 /detected_pillars: %zu 个柱子", detected_pillars_cm_.size());
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

  // 起飞航点只看高度
  if (current_idx_ == 0 && wp.z_cm > 20.0) {
    return z_ok;
  }
  // 高空航点要求 xy+z；低空（降落）再加 yaw
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
    "发布航点 %zu [%s]: x=%.1f y=%.1f z=%.1f yaw=%.1f",
    current_idx_, wp.tag, wp.x_cm, wp.y_cm, wp.z_cm, wp.yaw_deg);
}

void PillarScanMissionNode::publishEnable(bool on)
{
  std_msgs::msg::Bool msg;
  msg.data = on;
  enable_pub_->publish(msg);
  RCLCPP_INFO(get_logger(), "/pillar_detect_enable = %s", on ? "true" : "false");
}

std::vector<std::pair<double, double>> PillarScanMissionNode::greedyOrder(
  const std::vector<std::pair<double, double>> & pillars,
  double start_x_cm,
  double start_y_cm) const
{
  std::vector<std::pair<double, double>> remaining = pillars;
  std::vector<std::pair<double, double>> ordered;
  ordered.reserve(remaining.size());

  double cx = start_x_cm;
  double cy = start_y_cm;
  while (!remaining.empty()) {
    std::size_t best = 0;
    double best_d = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < remaining.size(); ++i) {
      const double dx = remaining[i].first - cx;
      const double dy = remaining[i].second - cy;
      const double d = std::hypot(dx, dy);
      if (d < best_d) { best_d = d; best = i; }
    }
    ordered.push_back(remaining[best]);
    cx = remaining[best].first;
    cy = remaining[best].second;
    remaining.erase(remaining.begin() + static_cast<std::ptrdiff_t>(best));
  }
  return ordered;
}

void PillarScanMissionNode::buildVisitAndLandWaypoints(double cur_x_cm, double cur_y_cm)
{
  // 在贪心排序之前先算出访问顺序（起点是当前 xy，不是爬升后的位置——xy 没变）
  const auto ordered = greedyOrder(detected_pillars_cm_, cur_x_cm, cur_y_cm);

  // 1) 原地爬升到访问高度（pillar_visit_height_cm，默认 150cm，高过所有柱子）
  waypoints_.push_back(ScanWaypoint{
    cur_x_cm, cur_y_cm,
    pillar_visit_height_cm_, 0.0, 0.0, "climb"
  });

  // 2) 贪心最近邻逐个飞到柱子正上方，每个悬停 pillar_hover_sec
  for (const auto & p : ordered) {
    waypoints_.push_back(ScanWaypoint{
      p.first, p.second,
      pillar_visit_height_cm_, 0.0,
      pillar_hover_sec_, "pillar"
    });
  }

  // 3) 降落分三段：先到 B 上方高空 → 降到低空 → 贴地
  //    xy 与 z 分开调整，避免大斜率斜插降落
  waypoints_.push_back(ScanWaypoint{landing_x_cm_, landing_y_cm_, pillar_visit_height_cm_, 0.0, 0.0, "land_approach"});
  waypoints_.push_back(ScanWaypoint{landing_x_cm_, landing_y_cm_, flight_height_cm_,      0.0, 0.0, "land_hover"});
  waypoints_.push_back(ScanWaypoint{landing_x_cm_, landing_y_cm_, land_height_cm_,        0.0, 0.0, "land"});

  RCLCPP_INFO(get_logger(),
    "规划完成: climb(%.0fcm) + 访问柱子 %zu 个 + 降落 3 段 (总航点=%zu)",
    pillar_visit_height_cm_, ordered.size(), waypoints_.size());
  for (std::size_t i = 0; i < ordered.size(); ++i) {
    RCLCPP_INFO(get_logger(),
      "  访问 #%zu: 柱子 @ (%.1f, %.1f) cm",
      i + 1, ordered[i].first, ordered[i].second);
  }
}

void PillarScanMissionNode::advance()
{
  ++current_idx_;
  is_hovering_ = false;
  if (current_idx_ < waypoints_.size()) {
    publishTarget(waypoints_[current_idx_]);
  } else {
    phase_ = MissionPhase::DONE;
    if (!mission_complete_sent_) {
      std_msgs::msg::Empty m;
      mission_complete_pub_->publish(m);
      mission_complete_sent_ = true;
    }
    std_msgs::msg::UInt8 stop_msg;
    stop_msg.data = 3;
    active_controller_pub_->publish(stop_msg);
    RCLCPP_INFO(get_logger(), "任务全部航点完成。");
  }
}

void PillarScanMissionNode::monitorTimerCallback()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (phase_ == MissionPhase::DONE) {
    std_msgs::msg::UInt8 stop_msg;
    stop_msg.data = 3;
    active_controller_pub_->publish(stop_msg);
    return;
  }

  double x_cm = 0.0, y_cm = 0.0, yaw_deg = 0.0;
  if (!getCurrentPose(x_cm, y_cm, yaw_deg)) { return; }
  const double z_cm = has_height_ ? current_height_cm_ : 0.0;

  // ── WAIT_PILLARS：在扫描终点保持悬停，等 /detected_pillars ──
  if (phase_ == MissionPhase::WAIT_PILLARS) {
    const double elapsed = (now() - wait_pillars_start_time_).seconds();
    if (pillars_received_) {
      const bool has_pillars = !detected_pillars_cm_.empty();
      RCLCPP_INFO(get_logger(),
        "收到柱子数据 (%.1fs 内，%zu 个)，%s",
        elapsed, detected_pillars_cm_.size(),
        has_pillars ? "规划访问航线。" : "无柱子，直接飞向降落点。");
      buildVisitAndLandWaypoints(x_cm, y_cm);
      phase_ = has_pillars ? MissionPhase::VISIT : MissionPhase::LAND;
      advance();
      return;
    }
    if (elapsed > pillar_wait_timeout_sec_) {
      RCLCPP_WARN(get_logger(),
        "等待 /detected_pillars 超时 (%.1fs)，直接飞向降落点。", elapsed);
      buildVisitAndLandWaypoints(x_cm, y_cm);
      phase_ = MissionPhase::LAND;
      advance();
      return;
    }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "悬停等待柱子数据 %.1fs / %.1fs",
      elapsed, pillar_wait_timeout_sec_);
    return;
  }

  if (current_idx_ >= waypoints_.size()) {
    phase_ = MissionPhase::DONE;
    return;
  }

  const auto & wp = waypoints_[current_idx_];

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
    "航点 %zu [%s] 目标=(%.1f,%.1f,%.1f,%.1f) 当前=(%.1f,%.1f,%.1f,%.1f)",
    current_idx_, wp.tag, wp.x_cm, wp.y_cm, wp.z_cm, wp.yaw_deg,
    x_cm, y_cm, z_cm, yaw_deg);

  if (!isReached(wp, x_cm, y_cm, z_cm, yaw_deg)) {
    is_hovering_ = false;
    return;
  }

  // ── 到位后悬停计时 ──
  if (wp.hover_sec > 0.0) {
    if (!is_hovering_) {
      is_hovering_ = true;
      hover_start_time_ = now();
      RCLCPP_INFO(get_logger(),
        "航点 %zu [%s] 到达，悬停 %.1fs", current_idx_, wp.tag, wp.hover_sec);
    }
    if ((now() - hover_start_time_).seconds() < wp.hover_sec) {
      return;
    }
  }

  // ── SCAN 阶段的事件触发：起飞完成 → enable=true；扫描终点 → enable=false + 转 WAIT_PILLARS ──
  if (phase_ == MissionPhase::SCAN) {
    if (current_idx_ == 0) {
      publishEnable(true);
    } else if (current_idx_ == scan_end_idx_) {
      publishEnable(false);
      pillars_received_ = false;  // 丢弃旧数据，等新的一条
      detected_pillars_cm_.clear();
      phase_ = MissionPhase::WAIT_PILLARS;
      wait_pillars_start_time_ = now();
      RCLCPP_INFO(get_logger(), "扫描终点到达，等待 /detected_pillars ...");
      return;  // 不 advance，保持在 scan_end 悬停
    }
  }

  RCLCPP_INFO(get_logger(), "航点 %zu [%s] 完成。", current_idx_, wp.tag);
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
