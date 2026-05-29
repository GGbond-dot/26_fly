#include "spray_control_pkg/spray_mission_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace spray_control_pkg
{

using namespace std::chrono_literals;

SprayMissionNode::SprayMissionNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("spray_mission_node", options),
  phase_(MissionPhase::TAKEOFF),
  current_idx_(0),
  spray_active_(false),
  spray_blink_step_(-1),
  spray_frame_count_(0),
  spray_seen_green_(false),
  laser_on_(false),
  barcode_observe_active_(false),
  barcode_detected_(false),
  barcode_number_(0),
  pillar_received_(false),
  pillar_inserted_(false),
  pillar_x_m_(0.0),
  pillar_y_m_(0.0),
  has_spray_allowed_(false),
  latest_spray_allowed_(false),
  has_height_(false),
  current_height_cm_(0.0),
  mission_complete_sent_(false),
  first_publish_done_(false)
{
  // ── 参数 ──
  map_frame_   = declare_parameter<std::string>("map_frame", "map");
  // 注：与 PID/uart 一致，使用 laser_link 作为机体位姿 frame
  base_frame_  = declare_parameter<std::string>("base_frame", "laser_link");

  pos_tol_cm_    = declare_parameter<double>("pos_tol_cm", 12.0);
  yaw_tol_deg_   = declare_parameter<double>("yaw_tol_deg", 10.0);
  height_tol_cm_ = declare_parameter<double>("height_tol_cm", 10.0);

  flight_height_cm_ = declare_parameter<double>("flight_height_cm", 150.0);
  land_height_cm_   = declare_parameter<double>("land_height_cm", 0.0);
  home_x_cm_        = declare_parameter<double>("home_x_cm", 0.0);
  home_y_cm_        = declare_parameter<double>("home_y_cm", 0.0);

  spray_decision_timeout_sec_   = declare_parameter<double>("spray_decision_timeout_sec", 1.5);
  spray_data_stale_timeout_sec_ = declare_parameter<double>("spray_data_stale_timeout_sec", 0.5);
  spray_on_sec_                 = declare_parameter<double>("spray_on_sec", 0.3);
  spray_off_sec_                = declare_parameter<double>("spray_off_sec", 0.3);

  enable_barcode_task_      = declare_parameter<bool>("enable_barcode_task", false);
  pillar_left_offset_m_     = declare_parameter<double>("pillar_left_offset_m", 0.8);
  barcode_target_z_cm_      = declare_parameter<double>("barcode_target_z_cm", 105.0);
  barcode_wait_timeout_sec_ = declare_parameter<double>("barcode_wait_timeout_sec", 8.0);

  enable_circle_landing_    = declare_parameter<bool>("enable_circle_landing", false);
  // N×10cm 中的 N 取哪一位：默认末位（个位）
  circle_radius_digit_div_  = declare_parameter<double>("circle_radius_digit_div", 1.0);

  std::string height_topic = declare_parameter<std::string>("height_topic", "/height");

  // ── ROS 接口 ──
  auto durable_qos = rclcpp::QoS(10).reliable().transient_local();
  target_pub_            = create_publisher<std_msgs::msg::Float32MultiArray>("/target_position", durable_qos);
  active_controller_pub_ = create_publisher<std_msgs::msg::UInt8>("/active_controller", durable_qos);
  // 激光复用电磁铁链路（同 STM32 GPIO，0x33 帧）：1=开, 0=关
  electromagnet_pub_     = create_publisher<std_msgs::msg::UInt8>("/electromagnet_control", durable_qos);
  led_digit_pub_         = create_publisher<std_msgs::msg::UInt8>("/led_digit", rclcpp::QoS(10).reliable());
  mission_complete_pub_  = create_publisher<std_msgs::msg::Empty>("/mission_complete", rclcpp::QoS(10).reliable());

  height_sub_ = create_subscription<std_msgs::msg::Int16>(
    height_topic, rclcpp::QoS(10),
    std::bind(&SprayMissionNode::heightCallback, this, std::placeholders::_1));

  spray_allowed_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/spray_allowed", rclcpp::QoS(10),
    std::bind(&SprayMissionNode::sprayAllowedCallback, this, std::placeholders::_1));

  barcode_text_sub_ = create_subscription<std_msgs::msg::String>(
    "/barcode_text", rclcpp::QoS(10),
    std::bind(&SprayMissionNode::barcodeTextCallback, this, std::placeholders::_1));

  pillar_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
    "/detected_pillar",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&SprayMissionNode::pillarCallback, this, std::placeholders::_1));

  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  buildCoverageWaypoints();
  publishLaser(false);

  monitor_timer_ = create_wall_timer(
    50ms, std::bind(&SprayMissionNode::monitorTimerCallback, this));

  RCLCPP_INFO(get_logger(),
    "撒药任务节点已启动：%zu 个航点，巡航高度 %.0fcm，frame=%s，barcode_task=%s",
    waypoints_.size(), flight_height_cm_, base_frame_.c_str(),
    enable_barcode_task_ ? "on" : "off");
}

// ─── 订阅回调 ───────────────────────────────────────────────────────────────
void SprayMissionNode::heightCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  current_height_cm_ = static_cast<double>(msg->data);
  has_height_ = true;
}

void SprayMissionNode::sprayAllowedCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_spray_allowed_ = msg->data;
  has_spray_allowed_ = true;
  last_spray_allowed_stamp_ = this->now();
}

void SprayMissionNode::barcodeTextCallback(const std_msgs::msg::String::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (msg->data.empty()) return;
  latest_barcode_text_ = msg->data;
  barcode_detected_ = true;
  barcode_number_ = parseBarcodeNumber(msg->data);
  RCLCPP_INFO(get_logger(), "条码识别成功：'%s' → number=%d",
              msg->data.c_str(), barcode_number_);
}

void SprayMissionNode::pillarCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (msg->data.size() < 2) {
    RCLCPP_WARN(get_logger(), "/detected_pillar 数据维度不足，忽略");
    return;
  }
  pillar_x_m_ = static_cast<double>(msg->data[0]);
  pillar_y_m_ = static_cast<double>(msg->data[1]);
  pillar_received_ = true;
  RCLCPP_INFO(get_logger(), "收到杆塔位置：x=%.2fm y=%.2fm", pillar_x_m_, pillar_y_m_);

  if (enable_barcode_task_ && !pillar_inserted_) {
    insertBarcodeObserveWaypoint(pillar_x_m_, pillar_y_m_);
    pillar_inserted_ = true;
  }
}

// ─── 航点构建 ───────────────────────────────────────────────────────────────
//
// TODO(场地标定): green_blocks 真实坐标按图 1 实测填写。
//   作业区 400cm(Y) × 500cm(X)，50×50cm 区块编号 1~28。
//   起点 A=21。下面的占位仅用于状态机连通性测试。
//   发挥(1) 把被改为灰色的连续 3~4 个区块从列表剔除即可。
void SprayMissionNode::buildCoverageWaypoints()
{
  waypoints_.clear();

  const double z = flight_height_cm_;

  // 1) 起飞：原地升到巡航高度
  waypoints_.push_back({home_x_cm_, home_y_cm_, z, 0.0, false, false, 0, "takeoff"});

  // 2) 飞往起点区块 A(21)（占位坐标，需实测替换）
  const double a_x = declare_parameter<double>("block_a_x_cm", 75.0);
  const double a_y = declare_parameter<double>("block_a_y_cm", 350.0);
  waypoints_.push_back({a_x, a_y, z, 0.0, true, false, 21, "block_A"});

  // 3) 覆盖路径：从 green_blocks 参数读 [id,x,y, id,x,y, ...]
  //    没配置时退化为只撒起点区块，保证骨架可编译可跑通流程。
  std::vector<double> blocks =
    declare_parameter<std::vector<double>>("green_blocks", std::vector<double>{});
  for (std::size_t i = 0; i + 2 < blocks.size(); i += 3) {
    SprayWaypoint wp;
    wp.block_id = static_cast<int>(blocks[i]);
    wp.x_cm     = blocks[i + 1];
    wp.y_cm     = blocks[i + 2];
    wp.z_cm     = z;
    wp.yaw_deg  = 0.0;
    wp.spray    = true;
    wp.wait_barcode = false;
    wp.tag      = "block";
    waypoints_.push_back(wp);
  }

  // 4) 返航到起降点上空
  waypoints_.push_back({home_x_cm_, home_y_cm_, z, 0.0, false, false, 0, "return"});

  // 5) 降落（如启用了圆周降落 + 识到条码数字，prepareFinalLanding 会改写此航点）
  waypoints_.push_back({home_x_cm_, home_y_cm_, land_height_cm_, 0.0, false, false, 0, "land"});
}

void SprayMissionNode::insertBarcodeObserveWaypoint(double pillar_x_m, double pillar_y_m)
{
  // 观察点 = 杆塔左侧偏移 pillar_left_offset_m_，高度对准条码段中心
  SprayWaypoint wp;
  wp.x_cm        = meterToCm(pillar_x_m);
  wp.y_cm        = meterToCm(pillar_y_m + pillar_left_offset_m_);
  wp.z_cm        = barcode_target_z_cm_;
  wp.yaw_deg     = 0.0;
  wp.spray       = false;
  wp.wait_barcode = true;
  wp.block_id    = 0;
  wp.tag         = "barcode_observe";

  // 插入到 GOTO_A 之后（idx=2 位置），保证起飞 → block_A → barcode_observe → 覆盖
  if (waypoints_.size() >= 2) {
    waypoints_.insert(waypoints_.begin() + 2, wp);
    RCLCPP_INFO(get_logger(),
      "已插入条码观察航点 idx=2：(%.1f, %.1f, %.1f)cm",
      wp.x_cm, wp.y_cm, wp.z_cm);
  } else {
    waypoints_.push_back(wp);
  }
}

// ─── tf / 到达判定 ─────────────────────────────────────────────────────────
bool SprayMissionNode::getCurrentPose(double & x_cm, double & y_cm, double & yaw_deg)
{
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "tf 查询失败: %s", ex.what());
    return false;
  }
  x_cm = meterToCm(tf.transform.translation.x);
  y_cm = meterToCm(tf.transform.translation.y);
  yaw_deg = normalizeAngleDeg(tf2::getYaw(tf.transform.rotation) * 180.0 / M_PI);
  return true;
}

bool SprayMissionNode::isReached(const SprayWaypoint & wp, double x_cm, double y_cm,
                                 double z_cm, double yaw_deg) const
{
  const double dxy = std::hypot(wp.x_cm - x_cm, wp.y_cm - y_cm);
  const double dz  = std::fabs(wp.z_cm - z_cm);
  const double dyaw = std::fabs(normalizeAngleDeg(wp.yaw_deg - yaw_deg));
  return dxy <= pos_tol_cm_ && dz <= height_tol_cm_ && dyaw <= yaw_tol_deg_;
}

// ─── 发布 ─────────────────────────────────────────────────────────────────
void SprayMissionNode::publishTarget(const SprayWaypoint & wp)
{
  std_msgs::msg::Float32MultiArray msg;
  msg.data = {static_cast<float>(wp.x_cm), static_cast<float>(wp.y_cm),
              static_cast<float>(wp.z_cm), static_cast<float>(wp.yaw_deg)};
  target_pub_->publish(msg);

  std_msgs::msg::UInt8 active_msg;
  active_msg.data = 2;
  active_controller_pub_->publish(active_msg);
}

void SprayMissionNode::publishLaser(bool on)
{
  // 物理上激光接在电磁铁 GPIO，沿用 /electromagnet_control 帧 0x33。
  std_msgs::msg::UInt8 msg;
  msg.data = on ? 1 : 0;
  electromagnet_pub_->publish(msg);
  laser_on_ = on;
}

void SprayMissionNode::publishLedDigit(uint8_t digit)
{
  std_msgs::msg::UInt8 msg;
  msg.data = digit;
  led_digit_pub_->publish(msg);
  RCLCPP_INFO(get_logger(), "/led_digit 发布数字 %u", static_cast<unsigned>(digit));
}

// ─── 撒药子状态机（颜色门控） ───────────────────────────────────────────
//
// 流程：
//   到 spray 航点
//   → 等下一帧 /spray_allowed（不同于 spray_start_time_ 之前的旧帧）
//   → 采 3 帧 → 任一为 true 视作见绿
//     ├ 见绿：发激光开 → 0.3s 关 → 0.3s 开 → 0.3s 关，闪 2 次后 advance
//     ├ 3 帧全灰：跳过本格不撒（应对发挥(1) 灰格）
//     └ 1.5s 仍未采够 3 帧：判超时跳过
bool SprayMissionNode::runSprayWithColorGate()
{
  const rclcpp::Time now = this->now();

  if (!spray_active_) {
    spray_active_ = true;
    spray_blink_step_ = -1;
    spray_frame_count_ = 0;
    spray_seen_green_ = false;
    spray_start_time_ = now;
    last_sampled_allowed_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    RCLCPP_INFO(get_logger(), "spray 决策开始：航点 %zu", current_idx_);
    return false;
  }

  // 已在闪烁中：按 on/off 时间推进
  if (spray_blink_step_ >= 0) {
    const double dt = (now - spray_blink_edge_time_).seconds();
    if (spray_blink_step_ == 0 && dt >= spray_on_sec_) {
      publishLaser(false);
      spray_blink_step_ = 1;
      spray_blink_edge_time_ = now;
    } else if (spray_blink_step_ == 1 && dt >= spray_off_sec_) {
      publishLaser(true);
      spray_blink_step_ = 2;
      spray_blink_edge_time_ = now;
    } else if (spray_blink_step_ == 2 && dt >= spray_on_sec_) {
      publishLaser(false);
      spray_active_ = false;
      spray_blink_step_ = -1;
      RCLCPP_INFO(get_logger(), "spray 闪烁完成：航点 %zu", current_idx_);
      return true;
    }
    return false;
  }

  // 采样 /spray_allowed
  const bool has_arrival_frame =
    has_spray_allowed_ &&
    last_spray_allowed_stamp_.nanoseconds() != 0 &&
    (last_spray_allowed_stamp_ - spray_start_time_).seconds() >= 0.0;
  const bool has_unsampled_frame =
    has_arrival_frame &&
    (last_spray_allowed_stamp_ - last_sampled_allowed_time_).nanoseconds() > 0;
  const double elapsed = (now - spray_start_time_).seconds();

  if (!has_arrival_frame ||
      (now - last_spray_allowed_stamp_).seconds() > spray_data_stale_timeout_sec_)
  {
    if (elapsed >= spray_decision_timeout_sec_) {
      RCLCPP_WARN(get_logger(),
        "spray 决策超时（航点 %zu，仅采到 %d/%d 帧），跳过",
        current_idx_, spray_frame_count_, kSprayDecisionFrameCount);
      spray_active_ = false;
      return true;
    }
    return false;
  }

  if (!has_unsampled_frame) return false;

  ++spray_frame_count_;
  spray_seen_green_ = spray_seen_green_ || latest_spray_allowed_;
  last_sampled_allowed_time_ = last_spray_allowed_stamp_;
  RCLCPP_INFO(get_logger(),
    "spray 颜色采样 %d/%d 航点 %zu: %s",
    spray_frame_count_, kSprayDecisionFrameCount, current_idx_,
    latest_spray_allowed_ ? "green" : "not_green");

  if (spray_frame_count_ < kSprayDecisionFrameCount) return false;

  if (spray_seen_green_) {
    publishLaser(true);
    spray_blink_step_ = 0;
    spray_blink_edge_time_ = now;
    RCLCPP_INFO(get_logger(),
      "航点 %zu 见绿 → 开始闪 2 次（on %.1fs / off %.1fs）",
      current_idx_, spray_on_sec_, spray_off_sec_);
    return false;
  }

  RCLCPP_INFO(get_logger(),
    "航点 %zu 视野无绿（%d 帧全灰），跳过不撒",
    current_idx_, spray_frame_count_);
  spray_active_ = false;
  return true;
}

// ─── 条码观察子状态 ────────────────────────────────────────────────────────
bool SprayMissionNode::runBarcodeObserve()
{
  const rclcpp::Time now = this->now();

  if (!barcode_observe_active_) {
    barcode_observe_active_ = true;
    barcode_observe_start_time_ = now;
    barcode_detected_ = false;
    RCLCPP_INFO(get_logger(), "到达条码观察点，悬停等 /barcode_text");
    return false;
  }

  if (barcode_detected_ && barcode_number_ > 0) {
    publishLedDigit(static_cast<uint8_t>(barcode_number_ % 10));  // 默认显示个位
    barcode_observe_active_ = false;
    return true;
  }

  if ((now - barcode_observe_start_time_).seconds() >= barcode_wait_timeout_sec_) {
    RCLCPP_WARN(get_logger(),
      "条码识别超时（%.1fs），跳过观察点继续覆盖",
      barcode_wait_timeout_sec_);
    barcode_observe_active_ = false;
    return true;
  }

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
    "等待 /barcode_text…");
  return false;
}

// ─── 返航前最终降落航点装配（圆周或居中） ───────────────────────────────
void SprayMissionNode::prepareFinalLanding()
{
  if (!enable_circle_landing_ || barcode_number_ <= 0) return;
  if (waypoints_.empty()) return;

  // 找当前位置（用最近一次 tf 查询位置粗算最近圆周点）
  double cx_cm = home_x_cm_, cy_cm = home_y_cm_, cyaw = 0.0;
  getCurrentPose(cx_cm, cy_cm, cyaw);

  // N 取条码数字的个位（默认）；若 circle_radius_digit_div_ != 1.0 可改成整数
  const int n = (circle_radius_digit_div_ <= 1.5)
                  ? (barcode_number_ % 10)
                  : barcode_number_;
  const double r_cm = n * 10.0;

  // 最近圆周点 = home + r * (当前位置 - home 的单位向量)
  const double dx = cx_cm - home_x_cm_;
  const double dy = cy_cm - home_y_cm_;
  const double norm = std::hypot(dx, dy);
  const double ux = (norm > 1e-3) ? dx / norm : 1.0;
  const double uy = (norm > 1e-3) ? dy / norm : 0.0;
  const double tx = home_x_cm_ + r_cm * ux;
  const double ty = home_y_cm_ + r_cm * uy;

  // 改写最后两个航点：return → circle_pre → circle_land
  // 当前序列末尾是 [return, land]
  const std::size_t n_wp = waypoints_.size();
  if (n_wp < 2) return;

  // 把 "return" 改为 "circle_pre"（同高度先飞到圆周点上空）
  SprayWaypoint pre = waypoints_[n_wp - 2];
  pre.x_cm = tx; pre.y_cm = ty;
  pre.tag = "circle_pre";
  waypoints_[n_wp - 2] = pre;

  // 把 "land" 改为 "circle_land"
  SprayWaypoint land = waypoints_[n_wp - 1];
  land.x_cm = tx; land.y_cm = ty;
  land.z_cm = land_height_cm_;
  land.tag = "circle_land";
  waypoints_[n_wp - 1] = land;

  RCLCPP_INFO(get_logger(),
    "圆周降落：N=%d 半径=%.0fcm，目标(%.1f, %.1f)cm",
    n, r_cm, tx, ty);
}

// ─── 状态推进 ───────────────────────────────────────────────────────────
void SprayMissionNode::advance()
{
  resetSprayState();
  resetBarcodeState();

  if (current_idx_ + 1 < waypoints_.size()) {
    ++current_idx_;
    // 进入"return"或"land"航点前，决定要不要切圆周降落
    if (waypoints_[current_idx_].tag &&
        std::string(waypoints_[current_idx_].tag) == "return")
    {
      prepareFinalLanding();
    }
    publishTarget(waypoints_[current_idx_]);
    const auto & wp = waypoints_[current_idx_];
    RCLCPP_INFO(get_logger(),
      "→ 航点 %zu [%s] block=%d x=%.1f y=%.1f z=%.1f spray=%d barcode=%d",
      current_idx_, wp.tag, wp.block_id, wp.x_cm, wp.y_cm, wp.z_cm,
      wp.spray, wp.wait_barcode);
  } else {
    if (!mission_complete_sent_) {
      mission_complete_pub_->publish(std_msgs::msg::Empty());
      std_msgs::msg::UInt8 stop;
      stop.data = 0;
      active_controller_pub_->publish(stop);
      publishLaser(false);
      mission_complete_sent_ = true;
      phase_ = MissionPhase::DONE;
      RCLCPP_INFO(get_logger(), "撒药任务完成，已发布 /mission_complete");
    }
  }
}

void SprayMissionNode::resetSprayState()
{
  spray_active_ = false;
  spray_blink_step_ = -1;
  spray_frame_count_ = 0;
  spray_seen_green_ = false;
}

void SprayMissionNode::resetBarcodeState()
{
  barcode_observe_active_ = false;
  // 注意：barcode_detected_ / barcode_number_ 不在 advance 时清，
  // 留给 prepareFinalLanding 用。
}

// ─── 主循环 ───────────────────────────────────────────────────────────
void SprayMissionNode::monitorTimerCallback()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (phase_ == MissionPhase::DONE) return;
  if (!has_height_) return;

  // 如果开了条码任务但还没收到杆塔位置，先等一会儿再起飞
  if (enable_barcode_task_ && !pillar_received_ && !first_publish_done_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
      "等待 /detected_pillar 后再起飞…");
    return;
  }

  double x, y, yaw;
  if (!getCurrentPose(x, y, yaw)) return;

  if (!first_publish_done_) {
    publishTarget(waypoints_[current_idx_]);
    first_publish_done_ = true;
    return;
  }

  const SprayWaypoint & wp = waypoints_[current_idx_];
  if (!isReached(wp, x, y, current_height_cm_, yaw)) return;

  // 到点：依据航点标志走子状态
  if (wp.spray && !runSprayWithColorGate()) return;
  if (wp.wait_barcode && !runBarcodeObserve()) return;

  advance();
}

// ─── 工具函数 ───────────────────────────────────────────────────────────
double SprayMissionNode::normalizeAngleDeg(double angle_deg) const
{
  while (angle_deg > 180.0)  angle_deg -= 360.0;
  while (angle_deg < -180.0) angle_deg += 360.0;
  return angle_deg;
}

int SprayMissionNode::parseBarcodeNumber(const std::string & text) const
{
  // 题目要求 4 位数字。尝试整数解析；非数字字符直接返回 0。
  if (text.empty()) return 0;
  try {
    int n = std::stoi(text);
    if (n < 0) return 0;
    return n;
  } catch (...) {
    return 0;
  }
}

}  // namespace spray_control_pkg
