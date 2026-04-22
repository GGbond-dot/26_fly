#include "activity_control_pkg/pillar_pickup_mission_node.hpp"

#include <algorithm>
#include <angles/angles.h>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace activity_control_pkg
{

PillarPickupMissionNode::PillarPickupMissionNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("pillar_pickup_mission", options),
  phase_(PickupPhase::SCAN),
  scan_end_idx_(1),
  current_idx_(0),
  pillar_iter_(0),
  sub_(PickupSub::APPROACH),
  sub_is_hovering_(false),
  has_fine_(false),
  last_fine_dx_(0),
  last_fine_dy_(0),
  visual_takeover_active_(false),
  measured_pillar_height_cm_(0.0),
  has_measured_pillar_height_(false),
  pillars_received_(false),
  is_hovering_(false),
  has_area_height_(false),
  area_height_cm_(0.0),
  has_point_height_(false),
  point_height_cm_(0.0),
  mission_complete_sent_(false)
{
  map_frame_        = declare_parameter<std::string>("map_frame",        "map");
  laser_link_frame_ = declare_parameter<std::string>("laser_link_frame", "laser_link");

  pos_tol_cm_    = declare_parameter("position_tolerance_cm",  9.0);
  yaw_tol_deg_   = declare_parameter("yaw_tolerance_deg",      5.0);
  height_tol_cm_ = declare_parameter("height_tolerance_cm",   12.0);

  flight_height_cm_        = declare_parameter("flight_height_cm",        40.0);
  land_height_cm_          = declare_parameter("land_height_cm",           4.0);
  scan_end_x_cm_           = declare_parameter("scan_end_x_cm",          250.0);
  landing_x_cm_            = declare_parameter("landing_x_cm",           250.0);
  landing_y_cm_            = declare_parameter("landing_y_cm",          -250.0);
  pillar_visit_height_cm_  = declare_parameter("pillar_visit_height_cm", 150.0);
  pillar_wait_timeout_sec_ = declare_parameter("pillar_wait_timeout_sec", 3.0);

  visual_align1_timeout_sec_  = declare_parameter("visual_align1_timeout_sec", 4.0);
  visual_align2_timeout_sec_  = declare_parameter("visual_align2_timeout_sec", 1.5);
  visual_pixel_tol_           = declare_parameter("visual_pixel_tol_px",      15);
  visual_align_required_hits_ = declare_parameter("visual_align_required_hits", 3);
  visual_jump_px_             = declare_parameter("visual_jump_px",          100);
  visual_stale_sec_           = declare_parameter("visual_stale_sec",        0.5);
  cam_offset_dx_cm_ = declare_parameter("cam_offset_dx_cm", CAM_TO_POINT_LASER_DX_CM);
  cam_offset_dy_cm_ = declare_parameter("cam_offset_dy_cm", CAM_TO_POINT_LASER_DY_CM);

  sample_duration_sec_         = declare_parameter("sample_duration_sec",        2.5);
  sample_min_pillar_frames_    = declare_parameter("sample_min_pillar_frames",    8);
  sample_pillar_drop_thresh_cm_= declare_parameter("sample_pillar_drop_thresh_cm", 20.0);

  mid_clearance_cm_   = declare_parameter("mid_clearance_cm",   30.0);
  grab_clearance_cm_  = declare_parameter("grab_clearance_cm",   5.0);
  hover_grab_sec_     = declare_parameter("hover_grab_sec",      2.0);

  // ── SCAN 航点 ──
  waypoints_ = {
    PickupWaypoint{0.0,            0.0, flight_height_cm_, 0.0, 0.0, "takeoff"},
    PickupWaypoint{scan_end_x_cm_, 0.0, flight_height_cm_, 0.0, 0.0, "scan_end"},
  };
  scan_end_idx_ = 1;

  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  auto durable_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  target_pub_            = create_publisher<std_msgs::msg::Float32MultiArray>("/target_position", durable_qos);
  enable_pub_            = create_publisher<std_msgs::msg::Bool>("/pillar_detect_enable", durable_qos);
  visual_takeover_pub_   = create_publisher<std_msgs::msg::Bool>("/visual_takeover", durable_qos);
  active_controller_pub_ = create_publisher<std_msgs::msg::UInt8>("/active_controller", durable_qos);
  mission_complete_pub_  = create_publisher<std_msgs::msg::Empty>("/mission_complete", rclcpp::QoS(10).reliable());

  area_height_sub_ = create_subscription<std_msgs::msg::Int16>(
    "/laser_array/ground_height", rclcpp::QoS(10),
    std::bind(&PillarPickupMissionNode::areaHeightCallback, this, std::placeholders::_1));
  point_height_sub_ = create_subscription<std_msgs::msg::Int16>(
    "/height", rclcpp::QoS(10),
    std::bind(&PillarPickupMissionNode::pointHeightCallback, this, std::placeholders::_1));
  fine_data_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
    "/fine_data", rclcpp::QoS(10),
    std::bind(&PillarPickupMissionNode::fineDataCallback, this, std::placeholders::_1));

  auto pillars_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  pillars_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
    "/detected_pillars", pillars_qos,
    std::bind(&PillarPickupMissionNode::pillarsCallback, this, std::placeholders::_1));

  publishEnable(false);
  publishVisualTakeover(false);
  publishTarget(waypoints_[0]);

  monitor_timer_ = create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&PillarPickupMissionNode::monitorTimerCallback, this));

  RCLCPP_INFO(get_logger(),
    "PICKUP 任务启动: 起飞 → 扫描 → 每柱子(对准+测高+下降+悬停%.1fs) → 降落(%.0f,%.0f)",
    hover_grab_sec_, landing_x_cm_, landing_y_cm_);
  RCLCPP_INFO(get_logger(),
    "激光: 面阵base=%.1fcm 点阵base=%.1fcm mount_diff=%.1fcm",
    LASER_AREA_BASE_CM, LASER_POINT_BASE_CM, LASER_MOUNT_DIFF_CM);
}

// ─────────── 回调 ───────────

void PillarPickupMissionNode::areaHeightCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
  area_height_cm_ = static_cast<double>(msg->data);
  has_area_height_ = true;
}

void PillarPickupMissionNode::pointHeightCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
  point_height_cm_ = static_cast<double>(msg->data);
  has_point_height_ = true;
}

void PillarPickupMissionNode::pillarsCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  detected_pillars_cm_.clear();
  if (msg->data.size() % 2 != 0) {
    RCLCPP_WARN(get_logger(), "/detected_pillars 数据长度异常: %zu", msg->data.size());
    pillars_received_ = true;
    return;
  }
  for (std::size_t k = 0; k + 1 < msg->data.size(); k += 2) {
    detected_pillars_cm_.emplace_back(
      static_cast<double>(msg->data[k]) * 100.0,
      static_cast<double>(msg->data[k + 1]) * 100.0);
  }
  pillars_received_ = true;
  RCLCPP_INFO(get_logger(), "收到 /detected_pillars: %zu 个柱子", detected_pillars_cm_.size());
}

void PillarPickupMissionNode::fineDataCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
  if (msg->data.size() < 2) { return; }
  recordFineData(static_cast<int>(msg->data[0]), static_cast<int>(msg->data[1]));
}

// ─────────── 发布工具 ───────────

void PillarPickupMissionNode::publishTarget(const PickupWaypoint & wp)
{
  std_msgs::msg::Float32MultiArray msg;
  msg.data.resize(4);
  msg.data[0] = static_cast<float>(wp.x_cm);
  msg.data[1] = static_cast<float>(wp.y_cm);
  msg.data[2] = static_cast<float>(wp.z_cm);
  msg.data[3] = static_cast<float>(wp.yaw_deg);
  target_pub_->publish(msg);
  publishActiveController(2);
  RCLCPP_INFO(get_logger(),
    "发布目标 [%s]: x=%.1f y=%.1f z=%.1f yaw=%.1f",
    wp.tag, wp.x_cm, wp.y_cm, wp.z_cm, wp.yaw_deg);
}

void PillarPickupMissionNode::publishEnable(bool on)
{
  std_msgs::msg::Bool m;
  m.data = on;
  enable_pub_->publish(m);
}

void PillarPickupMissionNode::publishVisualTakeover(bool on)
{
  std_msgs::msg::Bool m;
  m.data = on;
  visual_takeover_pub_->publish(m);
  visual_takeover_active_ = on;
  RCLCPP_INFO(get_logger(), "/visual_takeover = %s", on ? "true" : "false");
}

void PillarPickupMissionNode::publishActiveController(uint8_t mode)
{
  std_msgs::msg::UInt8 m;
  m.data = mode;
  active_controller_pub_->publish(m);
}

// ─────────── 姿态 / 到位 ───────────

bool PillarPickupMissionNode::getCurrentPose(double & x_cm, double & y_cm, double & yaw_deg)
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
      "TF %s <- %s 失败: %s", map_frame_.c_str(), laser_link_frame_.c_str(), ex.what());
    return false;
  }
}

bool PillarPickupMissionNode::isReached(const PickupWaypoint & wp, double x_cm, double y_cm,
                                        double z_cm, double yaw_deg) const
{
  const double dxy = std::hypot(wp.x_cm - x_cm, wp.y_cm - y_cm);
  const double dz  = std::fabs(wp.z_cm - z_cm);
  const double dyaw = std::fabs(normalizeAngleDeg(wp.yaw_deg - yaw_deg));

  const bool xy_ok  = dxy <= pos_tol_cm_;
  const bool z_ok   = dz  <= height_tol_cm_;
  const bool yaw_ok = dyaw <= yaw_tol_deg_;

  if (wp.z_cm > 20.0) {
    return z_ok && xy_ok;
  }
  return z_ok && xy_ok && yaw_ok;
}

// ─────────── 视觉对准 ───────────

void PillarPickupMissionNode::recordFineData(int dx_px, int dy_px)
{
  last_fine_dx_ = dx_px;
  last_fine_dy_ = dy_px;
  has_fine_ = true;
  last_fine_time_ = now();
  fine_hist_.emplace_back(dx_px, dy_px);
  while (fine_hist_.size() > static_cast<std::size_t>(std::max(1, visual_align_required_hits_))) {
    fine_hist_.pop_front();
  }
}

bool PillarPickupMissionNode::isVisuallyAligned() const
{
  if (static_cast<int>(fine_hist_.size()) < visual_align_required_hits_) { return false; }
  for (const auto & p : fine_hist_) {
    if (std::abs(p.first)  > visual_pixel_tol_) { return false; }
    if (std::abs(p.second) > visual_pixel_tol_) { return false; }
  }
  return true;
}

// ─────────── 高度采样 ───────────

void PillarPickupMissionNode::tryAccumulateHeightSample()
{
  if (!has_area_height_ || !has_point_height_) { return; }
  // 预期点阵打到地面的读数：h_point_expected = h_area - mount_diff
  const double h_point_expected = area_height_cm_ - LASER_MOUNT_DIFF_CM;
  const double drop = h_point_expected - point_height_cm_;
  if (drop >= sample_pillar_drop_thresh_cm_) {
    // 点阵这一帧打在柱子上 → 柱子高度样本 = drop
    pillar_height_samples_cm_.push_back(drop);
  }
}

bool PillarPickupMissionNode::finalizePillarHeight(double & out_cm) const
{
  if (static_cast<int>(pillar_height_samples_cm_.size()) < sample_min_pillar_frames_) {
    return false;
  }
  auto v = pillar_height_samples_cm_;
  std::sort(v.begin(), v.end());
  // 截断 10% 头尾取均值
  const std::size_t n = v.size();
  const std::size_t trim = std::max<std::size_t>(1, n / 10);
  if (n <= 2 * trim) {
    out_cm = v[n / 2];
    return true;
  }
  double sum = std::accumulate(v.begin() + trim, v.end() - trim, 0.0);
  out_cm = sum / static_cast<double>(n - 2 * trim);
  return true;
}

// ─────────── 摄像头偏置补偿 ───────────

void PillarPickupMissionNode::applyCameraOffsetToTarget(
  double & x_cm, double & y_cm, double yaw_deg) const
{
  // 摄像头在机体系相对点阵激光的偏置 (dx, dy)。
  // 当视觉把摄像头对到方框中心正上方时，carto 定位的 laser_link 其实偏离了柱子中心 -(dx, dy)（机体系）。
  // 飞到"柱子正上方"即把 laser_link 目标加一个 +机体系(dx,dy) 的偏置，等价于在 map 系旋转 yaw 后加上。
  const double c = std::cos(degToRad(yaw_deg));
  const double s = std::sin(degToRad(yaw_deg));
  const double ox = cam_offset_dx_cm_;
  const double oy = cam_offset_dy_cm_;
  x_cm += c * ox - s * oy;
  y_cm += s * ox + c * oy;
}

// ─────────── 贪心排序 ───────────

std::vector<std::pair<double, double>> PillarPickupMissionNode::greedyOrder(
  const std::vector<std::pair<double, double>> & pillars,
  double start_x_cm, double start_y_cm) const
{
  std::vector<std::pair<double, double>> remaining = pillars;
  std::vector<std::pair<double, double>> ordered;
  ordered.reserve(remaining.size());
  double cx = start_x_cm, cy = start_y_cm;
  while (!remaining.empty()) {
    std::size_t best = 0;
    double best_d = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < remaining.size(); ++i) {
      const double d = std::hypot(remaining[i].first - cx, remaining[i].second - cy);
      if (d < best_d) { best_d = d; best = i; }
    }
    ordered.push_back(remaining[best]);
    cx = remaining[best].first;
    cy = remaining[best].second;
    remaining.erase(remaining.begin() + static_cast<std::ptrdiff_t>(best));
  }
  return ordered;
}

// ─────────── 子阶段切换 ───────────

void PillarPickupMissionNode::enterSub(PickupSub s)
{
  sub_ = s;
  sub_enter_time_ = now();
  sub_is_hovering_ = false;
  fine_hist_.clear();

  const auto & p = pillars_ordered_cm_[pillar_iter_];
  double px = p.first, py = p.second;

  switch (s) {
    case PickupSub::APPROACH: {
      sub_target_ = PickupWaypoint{px, py, pillar_visit_height_cm_, 0.0, 0.0, "approach"};
      publishVisualTakeover(false);
      publishTarget(sub_target_);
      pillar_height_samples_cm_.clear();
      has_measured_pillar_height_ = false;
      break;
    }
    case PickupSub::ALIGN1: {
      // 保持 xy 目标不变（carto 仍做粗闭环），同时开启视觉接管
      sub_target_ = PickupWaypoint{px, py, pillar_visit_height_cm_, 0.0, 0.0, "align1"};
      publishTarget(sub_target_);
      pillar_height_samples_cm_.clear();
      sample_start_time_ = now();
      publishVisualTakeover(true);
      break;
    }
    case PickupSub::DESCEND_MID: {
      publishVisualTakeover(false);
      const double z_mid = measured_pillar_height_cm_
                         + LASER_AREA_BASE_CM + mid_clearance_cm_;
      double tx = px, ty = py;
      applyCameraOffsetToTarget(tx, ty, 0.0);
      sub_target_ = PickupWaypoint{tx, ty, z_mid, 0.0, 0.0, "descend_mid"};
      publishTarget(sub_target_);
      break;
    }
    case PickupSub::ALIGN2: {
      publishVisualTakeover(true);
      // 目标沿用上一段位置，只保持悬停
      break;
    }
    case PickupSub::DESCEND_FINAL: {
      publishVisualTakeover(false);
      const double z_grab = measured_pillar_height_cm_
                          + LASER_AREA_BASE_CM + grab_clearance_cm_;
      sub_target_.z_cm = z_grab;
      sub_target_.tag  = "descend_final";
      publishTarget(sub_target_);
      break;
    }
    case PickupSub::HOVER_GRAB: {
      sub_hover_start_ = now();
      RCLCPP_INFO(get_logger(),
        "[pillar %zu] HOVER_GRAB 占位（抓取/放置）: 悬停 %.1fs",
        pillar_iter_ + 1, hover_grab_sec_);
      // TODO: 在此处触发机械臂 / 电磁铁抓取；当前仅悬停
      break;
    }
    case PickupSub::CLIMB_BACK: {
      publishVisualTakeover(false);
      sub_target_ = PickupWaypoint{px, py, pillar_visit_height_cm_, 0.0, 0.0, "climb_back"};
      publishTarget(sub_target_);
      break;
    }
  }
  RCLCPP_INFO(get_logger(),
    "[pillar %zu] 进入子阶段 %d", pillar_iter_ + 1, static_cast<int>(s));
}

// ─────────── 每柱子主逻辑 ───────────

void PillarPickupMissionNode::stepPickup(double x_cm, double y_cm, double z_cm, double yaw_deg)
{
  (void)yaw_deg;
  // fine_data 看门狗：连续 > visual_stale_sec 没更新 或 跳变过大 → 关 takeover
  if (visual_takeover_active_ && has_fine_) {
    const double age = (now() - last_fine_time_).seconds();
    if (age > visual_stale_sec_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "fine_data stale %.2fs, 视觉不可用（暂不关 takeover，让 PID 内部保持）", age);
    }
  }

  switch (sub_) {
    case PickupSub::APPROACH: {
      if (isReached(sub_target_, x_cm, y_cm, z_cm, 0.0)) {
        enterSub(PickupSub::ALIGN1);
      }
      break;
    }
    case PickupSub::ALIGN1: {
      tryAccumulateHeightSample();
      const double elapsed = (now() - sub_enter_time_).seconds();
      const bool aligned = isVisuallyAligned();
      const bool samples_ok =
        static_cast<int>(pillar_height_samples_cm_.size()) >= sample_min_pillar_frames_;

      // 到期判定：达到对准 + 样本 OK → 提前结束；否则等到 timeout
      if ((aligned && samples_ok) || elapsed > visual_align1_timeout_sec_) {
        if (!finalizePillarHeight(measured_pillar_height_cm_)) {
          RCLCPP_WARN(get_logger(),
            "[pillar %zu] 采样柱子帧不足 (%zu)，本柱跳过",
            pillar_iter_ + 1, pillar_height_samples_cm_.size());
          // 跳过本柱子
          publishVisualTakeover(false);
          ++pillar_iter_;
          if (pillar_iter_ >= pillars_ordered_cm_.size()) {
            phase_ = PickupPhase::LAND;
            current_idx_ = waypoints_.size();  // LAND 段会 append
            // 构造降落航点
            waypoints_.push_back(PickupWaypoint{landing_x_cm_, landing_y_cm_, pillar_visit_height_cm_, 0.0, 0.0, "land_approach"});
            waypoints_.push_back(PickupWaypoint{landing_x_cm_, landing_y_cm_, flight_height_cm_,       0.0, 0.0, "land_hover"});
            waypoints_.push_back(PickupWaypoint{landing_x_cm_, landing_y_cm_, land_height_cm_,         0.0, 0.0, "land"});
            publishTarget(waypoints_[current_idx_]);
          } else {
            enterSub(PickupSub::APPROACH);
          }
          return;
        }
        has_measured_pillar_height_ = true;
        RCLCPP_INFO(get_logger(),
          "[pillar %zu] 测得高台高度 = %.1fcm（样本 %zu 帧, 对准=%s, 用时 %.1fs）",
          pillar_iter_ + 1, measured_pillar_height_cm_,
          pillar_height_samples_cm_.size(),
          aligned ? "达成" : "超时",
          elapsed);
        enterSub(PickupSub::DESCEND_MID);
      }
      break;
    }
    case PickupSub::DESCEND_MID: {
      if (isReached(sub_target_, x_cm, y_cm, z_cm, 0.0)) {
        enterSub(PickupSub::ALIGN2);
      }
      break;
    }
    case PickupSub::ALIGN2: {
      const double elapsed = (now() - sub_enter_time_).seconds();
      if (elapsed > visual_align2_timeout_sec_) {
        // best-effort: 不要求对准，直接下降
        enterSub(PickupSub::DESCEND_FINAL);
      }
      break;
    }
    case PickupSub::DESCEND_FINAL: {
      if (isReached(sub_target_, x_cm, y_cm, z_cm, 0.0)) {
        enterSub(PickupSub::HOVER_GRAB);
      }
      break;
    }
    case PickupSub::HOVER_GRAB: {
      if ((now() - sub_hover_start_).seconds() >= hover_grab_sec_) {
        // TODO: 抓取/放置确认
        enterSub(PickupSub::CLIMB_BACK);
      }
      break;
    }
    case PickupSub::CLIMB_BACK: {
      if (isReached(sub_target_, x_cm, y_cm, z_cm, 0.0)) {
        ++pillar_iter_;
        if (pillar_iter_ >= pillars_ordered_cm_.size()) {
          phase_ = PickupPhase::LAND;
          current_idx_ = waypoints_.size();
          waypoints_.push_back(PickupWaypoint{landing_x_cm_, landing_y_cm_, pillar_visit_height_cm_, 0.0, 0.0, "land_approach"});
          waypoints_.push_back(PickupWaypoint{landing_x_cm_, landing_y_cm_, flight_height_cm_,       0.0, 0.0, "land_hover"});
          waypoints_.push_back(PickupWaypoint{landing_x_cm_, landing_y_cm_, land_height_cm_,         0.0, 0.0, "land"});
          publishTarget(waypoints_[current_idx_]);
        } else {
          enterSub(PickupSub::APPROACH);
        }
      }
      break;
    }
  }
}

// ─────────── 主循环 ───────────

void PillarPickupMissionNode::monitorTimerCallback()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (phase_ == PickupPhase::DONE) {
    publishActiveController(3);
    return;
  }

  double x_cm = 0.0, y_cm = 0.0, yaw_deg = 0.0;
  if (!getCurrentPose(x_cm, y_cm, yaw_deg)) { return; }
  const double z_cm = has_area_height_ ? area_height_cm_ : 0.0;

  // ── WAIT_PILLARS ──
  if (phase_ == PickupPhase::WAIT_PILLARS) {
    const double elapsed = (now() - wait_pillars_start_time_).seconds();
    if (pillars_received_) {
      pillars_ordered_cm_ = greedyOrder(detected_pillars_cm_, x_cm, y_cm);
      RCLCPP_INFO(get_logger(),
        "柱子顺序已规划: %zu 个", pillars_ordered_cm_.size());
      if (pillars_ordered_cm_.empty()) {
        phase_ = PickupPhase::LAND;
        current_idx_ = waypoints_.size();
        waypoints_.push_back(PickupWaypoint{landing_x_cm_, landing_y_cm_, pillar_visit_height_cm_, 0.0, 0.0, "land_approach"});
        waypoints_.push_back(PickupWaypoint{landing_x_cm_, landing_y_cm_, flight_height_cm_,       0.0, 0.0, "land_hover"});
        waypoints_.push_back(PickupWaypoint{landing_x_cm_, landing_y_cm_, land_height_cm_,         0.0, 0.0, "land"});
        publishTarget(waypoints_[current_idx_]);
      } else {
        phase_ = PickupPhase::PICKUP;
        pillar_iter_ = 0;
        enterSub(PickupSub::APPROACH);
      }
      return;
    }
    if (elapsed > pillar_wait_timeout_sec_) {
      RCLCPP_WARN(get_logger(), "等待 /detected_pillars 超时 %.1fs，直接降落", elapsed);
      phase_ = PickupPhase::LAND;
      current_idx_ = waypoints_.size();
      waypoints_.push_back(PickupWaypoint{landing_x_cm_, landing_y_cm_, pillar_visit_height_cm_, 0.0, 0.0, "land_approach"});
      waypoints_.push_back(PickupWaypoint{landing_x_cm_, landing_y_cm_, flight_height_cm_,       0.0, 0.0, "land_hover"});
      waypoints_.push_back(PickupWaypoint{landing_x_cm_, landing_y_cm_, land_height_cm_,         0.0, 0.0, "land"});
      publishTarget(waypoints_[current_idx_]);
      return;
    }
    return;
  }

  // ── PICKUP ──
  if (phase_ == PickupPhase::PICKUP) {
    stepPickup(x_cm, y_cm, z_cm, yaw_deg);
    return;
  }

  // ── SCAN / LAND：航点式推进 ──
  if (current_idx_ >= waypoints_.size()) {
    phase_ = PickupPhase::DONE;
    if (!mission_complete_sent_) {
      std_msgs::msg::Empty e;
      mission_complete_pub_->publish(e);
      mission_complete_sent_ = true;
    }
    publishActiveController(3);
    return;
  }

  const auto & wp = waypoints_[current_idx_];

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
    "航点 %zu [%s] 目标=(%.1f,%.1f,%.1f) 当前=(%.1f,%.1f,%.1f)",
    current_idx_, wp.tag, wp.x_cm, wp.y_cm, wp.z_cm,
    x_cm, y_cm, z_cm);

  if (!isReached(wp, x_cm, y_cm, z_cm, yaw_deg)) {
    is_hovering_ = false;
    return;
  }

  if (wp.hover_sec > 0.0) {
    if (!is_hovering_) {
      is_hovering_ = true;
      hover_start_time_ = now();
    }
    if ((now() - hover_start_time_).seconds() < wp.hover_sec) { return; }
  }

  // SCAN 阶段事件
  if (phase_ == PickupPhase::SCAN) {
    if (current_idx_ == 0) {
      publishEnable(true);
    } else if (current_idx_ == scan_end_idx_) {
      publishEnable(false);
      pillars_received_ = false;
      detected_pillars_cm_.clear();
      phase_ = PickupPhase::WAIT_PILLARS;
      wait_pillars_start_time_ = now();
      RCLCPP_INFO(get_logger(), "扫描终点到达，等待 /detected_pillars ...");
      return;
    }
  }

  RCLCPP_INFO(get_logger(), "航点 %zu [%s] 完成。", current_idx_, wp.tag);
  ++current_idx_;
  is_hovering_ = false;
  if (current_idx_ < waypoints_.size()) {
    publishTarget(waypoints_[current_idx_]);
  } else {
    phase_ = PickupPhase::DONE;
    if (!mission_complete_sent_) {
      std_msgs::msg::Empty e;
      mission_complete_pub_->publish(e);
      mission_complete_sent_ = true;
    }
    publishActiveController(3);
    RCLCPP_INFO(get_logger(), "任务全部完成。");
  }
}

// ─────────── 工具 ───────────

double PillarPickupMissionNode::radToDeg(double v)
{
  return v * 180.0 / M_PI;
}

double PillarPickupMissionNode::normalizeAngleDeg(double angle_deg) const
{
  const double n = angles::normalize_angle(angles::from_degrees(angle_deg));
  return angles::to_degrees(n);
}

}  // namespace activity_control_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<activity_control_pkg::PillarPickupMissionNode>());
  rclcpp::shutdown();
  return 0;
}
