#include "fire_control_pkg/fire_mission_node.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace fire_control_pkg
{

using namespace std::chrono_literals;

FireMissionNode::FireMissionNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("fire_mission_node", options),
  phase_(MissionPhase::WAIT_START),
  current_idx_(0),
  fire_wp_{0.0, 0.0, 0.0, 0.0, -1, "fire"},
  use_fire_wp_(false),
  patrol_resume_idx_(0),
  fire_handled_(false),
  fire_confirmed_(false),
  fire_reported_(false),
  has_fire_position_(false),
  fire_x_dm_(0.0),
  fire_y_dm_(0.0),
  fire_candidate_x_dm_(0.0),
  fire_candidate_y_dm_(0.0),
  latest_fire_detected_(false),
  has_servo_error_(false),
  servo_ex_(0.0),
  servo_ey_(0.0),
  servo_bx_cm_(0.0),
  servo_by_cm_(0.0),
  servo_timed_out_(false),
  drop_sent_(false),
  patrol_distance_dm_(0.0),
  has_last_sample_(false),
  last_sample_x_dm_(0.0),
  last_sample_y_dm_(0.0),
  has_height_(false),
  current_height_dm_(0.0),
  mission_complete_sent_(false),
  first_publish_done_(false),
  status_inited_(false)
{
  // ── 参数 ──
  map_frame_  = declare_parameter<std::string>("map_frame", "map");
  // 与 PID/uart 一致，用 laser_link 作为机体位姿 frame
  base_frame_ = declare_parameter<std::string>("base_frame", "laser_link");

  // 起飞点在场地系下的位置：题目图 1 左下黑色区 11dm×7dm 的中心
  // 起降区实测（照题目图 1 反算像素）：黑块 x 0~7 / y 0~7，即 7×7dm，中心 (3.5, 3.5)。
  // 图上那个 "11dm" 标注量的是原点到红色停车区左沿，不是黑块宽度——别再按 11×7 算中心。
  home_field_x_dm_     = declare_parameter<double>("home_field_x_dm", 3.5);
  home_field_y_dm_     = declare_parameter<double>("home_field_y_dm", 3.5);
  field_yaw_offset_deg_ = declare_parameter<double>("field_yaw_offset_deg", 0.0);

  pos_tol_dm_    = declare_parameter<double>("pos_tol_dm", 1.5);
  yaw_tol_deg_   = declare_parameter<double>("yaw_tol_deg", 10.0);
  height_tol_dm_ = declare_parameter<double>("height_tol_dm", 1.5);

  patrol_height_dm_ = declare_parameter<double>("patrol_height_dm", 18.0);
  drop_height_dm_   = declare_parameter<double>("drop_height_dm", 10.0);
  land_height_dm_   = declare_parameter<double>("land_height_dm", 0.0);

  // 覆盖宽度 8dm → 40dm 高的区域分 5 条带，中心线 y = 4/12/20/28/36。
  // x 两端各留 2dm 余量：激光扫到 8dm×8dm 框内任意一点即算覆盖该格，不必飞到边界线上。
  strip_y_dm_       = declare_parameter<std::vector<double>>(
    "strip_y_dm", std::vector<double>{4.0, 12.0, 20.0, 28.0, 36.0});
  // 覆盖足迹 8dm×8dm 以机身为中心 → 飞到 x=4 时左沿正好压到 x=0，飞到 x=44 右沿压到 x=48。
  // 再往外飞足迹已经出界，是纯浪费；同理 y 的 4 和 36 也正好内缩 4dm，一格不多一格不少。
  patrol_x_min_dm_  = declare_parameter<double>("patrol_x_min_dm", 4.0);
  patrol_x_max_dm_  = declare_parameter<double>("patrol_x_max_dm", 44.0);
  patrol_yaw_deg_   = declare_parameter<double>("patrol_yaw_deg", 0.0);

  enable_fire_task_       = declare_parameter<bool>("enable_fire_task", false);
  fire_confirm_sec_       = declare_parameter<double>("fire_confirm_sec", 0.8);
  fire_position_jitter_dm_ = declare_parameter<double>("fire_position_jitter_dm", 2.0);
  hover_before_drop_sec_  = declare_parameter<double>("hover_before_drop_sec", 3.0);
  drop_pulse_sec_         = declare_parameter<double>("drop_pulse_sec", 1.0);
  servo_deadband_ratio_   = declare_parameter<double>("servo_deadband_ratio", 0.05);
  servo_gain_             = declare_parameter<double>("servo_gain", 0.6);
  servo_max_step_dm_      = declare_parameter<double>("servo_max_step_dm", 5.0);
  servo_timeout_sec_      = declare_parameter<double>("servo_timeout_sec", 12.0);
  servo_data_timeout_sec_ = declare_parameter<double>("servo_data_timeout_sec", 1.0);
  // 注：火源上报的丢包冗余在 fire_link_pkg 的 report_repeat 参数里，本包不再重复发。

  dist_sample_min_dm_ = declare_parameter<double>("dist_sample_min_dm", 0.5);

  const std::string height_topic = declare_parameter<std::string>("height_topic", "/height");
  const bool auto_start = declare_parameter<bool>("auto_start", false);

  // ── ROS 接口 ──
  auto durable_qos = rclcpp::QoS(10).reliable().transient_local();
  target_pub_            = create_publisher<std_msgs::msg::Float32MultiArray>("/target_position", durable_qos);
  active_controller_pub_ = create_publisher<std_msgs::msg::UInt8>("/active_controller", durable_qos);
  // uart_to_stm32 的速度转发门控：发合法 /route_choice(=1) 才会把 /target_velocity 下发飞控
  route_choice_pub_      = create_publisher<std_msgs::msg::UInt8>("/route_choice", durable_qos);
  // 机腹激光笔走香橙派 40pin(PB0)，由 magnet_control_pkg 的 Python 节点执行 WiringOP
  // `gpio write`（低电平点亮）。本题激光只指示航迹，起飞常亮、降落熄灭。
  laser_pub_             = create_publisher<std_msgs::msg::Int32>("/magnet/cmd", durable_qos);
  alarm_led_pub_         = create_publisher<std_msgs::msg::UInt8>("/buzzer_led_control", rclcpp::QoS(10).reliable());
  drop_pub_              = create_publisher<std_msgs::msg::UInt8>("/drop_package", rclcpp::QoS(10).reliable());
  drone_pose_pub_        = create_publisher<std_msgs::msg::Float32MultiArray>("/drone_pose", rclcpp::QoS(10));
  patrol_distance_pub_   = create_publisher<std_msgs::msg::Float32MultiArray>("/patrol_distance", rclcpp::QoS(10));
  fire_report_pub_       = create_publisher<std_msgs::msg::Float32MultiArray>("/fire_report", durable_qos);
  fire_status_pub_       = create_publisher<std_msgs::msg::String>("/fire_status", rclcpp::QoS(10));
  mission_complete_pub_  = create_publisher<std_msgs::msg::Empty>("/mission_complete", rclcpp::QoS(10).reliable());

  // 消防车按键启动。transient_local：车先发、飞机后起也能接到。
  start_sub_ = create_subscription<std_msgs::msg::Empty>(
    "/fire_start", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
    std::bind(&FireMissionNode::startCallback, this, std::placeholders::_1));

  height_sub_ = create_subscription<std_msgs::msg::Int16>(
    height_topic, rclcpp::QoS(10),
    std::bind(&FireMissionNode::heightCallback, this, std::placeholders::_1));

  fire_detected_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/fire/detected", rclcpp::QoS(10),
    std::bind(&FireMissionNode::fireDetectedCallback, this, std::placeholders::_1));

  fire_position_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
    "/fire/position_map", rclcpp::QoS(10),
    std::bind(&FireMissionNode::firePositionCallback, this, std::placeholders::_1));

  // 视觉伺服误差：每帧一条，只在 APPROACH / SERVO_FINE 用。QoS 保留最新一条即可，
  // 队列积压反而会拿旧误差去修正。
  servo_error_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
    "/fire/servo_error", rclcpp::QoS(1),
    std::bind(&FireMissionNode::servoErrorCallback, this, std::placeholders::_1));

  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  fire_candidate_since_  = this->now();
  servo_error_time_      = this->now();
  phase_start_time_      = this->now();
  drop_sent_time_        = this->now();
  last_status_pub_time_  = this->now();

  buildPatrolWaypoints();
  publishLaser(false);
  publishAlarmLed(false);

  monitor_timer_ = create_wall_timer(
    50ms, std::bind(&FireMissionNode::monitorTimerCallback, this));
  // 基本要求(3)：巡逻期间每秒向消防车发 1 次位置坐标
  telemetry_timer_ = create_wall_timer(
    1000ms, std::bind(&FireMissionNode::telemetryTimerCallback, this));

  if (auto_start) {
    phase_ = MissionPhase::TAKEOFF;
    RCLCPP_WARN(get_logger(), "auto_start=true：跳过等待消防车按键，直接起飞（仅调试用）");
  }

  RCLCPP_INFO(get_logger(),
    "消防任务节点已启动：%zu 个航点，%zu 条巡逻带，巡逻高度 %.0fdm，frame=%s，fire_task=%s",
    waypoints_.size(), strip_y_dm_.size(), patrol_height_dm_, base_frame_.c_str(),
    enable_fire_task_ ? "on" : "off");
}

// ─── 订阅回调 ───────────────────────────────────────────────────────────────
void FireMissionNode::startCallback(const std_msgs::msg::Empty::SharedPtr)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (phase_ != MissionPhase::WAIT_START) return;   // 只认第一次，防重复触发
  phase_ = MissionPhase::TAKEOFF;
  phase_start_time_ = this->now();
  RCLCPP_INFO(get_logger(), "收到消防车启动信号 /fire_start → 起飞");
}

void FireMissionNode::heightCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  current_height_dm_ = cmToDm(static_cast<double>(msg->data));
  has_height_ = true;
}

void FireMissionNode::fireDetectedCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_fire_detected_ = msg->data;
}

// fire_vision 发的是 **map 系 cm**；这里换成场地系 dm 再做稳定性判定。
// 判定通过才锁定坐标——vision 侧已有多帧投票，这里再卡一层位置抖动，
// 避免飞机高速掠过时反投影误差把火源定偏（半径 3dm 的抛包判定不容偏太多）。
void FireMissionNode::firePositionCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!enable_fire_task_ || fire_handled_ || fire_confirmed_) return;
  if (msg->data.size() < 2) return;

  double x_dm, y_dm;
  mapToField(static_cast<double>(msg->data[0]), static_cast<double>(msg->data[1]), x_dm, y_dm);

  const rclcpp::Time now = this->now();
  if (!has_fire_position_ ||
      std::hypot(x_dm - fire_candidate_x_dm_, y_dm - fire_candidate_y_dm_) > fire_position_jitter_dm_)
  {
    // 首次收到，或跳变超出抖动容许 → 重新起算
    fire_candidate_x_dm_ = x_dm;
    fire_candidate_y_dm_ = y_dm;
    fire_candidate_since_ = now;
    has_fire_position_ = true;
    return;
  }

  // 位置稳定，滑动平均收敛到候选点
  fire_candidate_x_dm_ = 0.5 * (fire_candidate_x_dm_ + x_dm);
  fire_candidate_y_dm_ = 0.5 * (fire_candidate_y_dm_ + y_dm);

  if ((now - fire_candidate_since_).seconds() >= fire_confirm_sec_) {
    fire_x_dm_ = fire_candidate_x_dm_;
    fire_y_dm_ = fire_candidate_y_dm_;
    fire_confirmed_ = true;
    RCLCPP_INFO(get_logger(), "火源已确认：反投影坐标 (%.1f, %.1f) dm（仅用于触发，不上报）",
      fire_x_dm_, fire_y_dm_);
    // 这里**不上报**。反投影坐标受 fx/fy、/height、cam_offset 影响，误差 2~3dm，
    // 而车端拿它判"落在哪个街区"，出框就直接拒绝出发。
    // 改为闭环对中完成后用飞机**自身位姿**上报（见 APPROACH 分支），
    // 那时误差只剩 Cartographer 本身，且仍在降高/抛包之前，车不会被拖慢。
  }
}

void FireMissionNode::servoErrorCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (msg->data.size() < 4) return;
  servo_ex_         = static_cast<double>(msg->data[0]);
  servo_ey_         = static_cast<double>(msg->data[1]);
  servo_bx_cm_      = static_cast<double>(msg->data[2]);
  servo_by_cm_      = static_cast<double>(msg->data[3]);
  servo_error_time_ = this->now();
  has_servo_error_  = true;
}

// 视觉伺服一步。相机与抛投口视为同 z 轴，所以对中目标就是画面中心。
//
// fire_vision 给的 (bx, by) 已经是**机体系**的地面偏移（+x 前 / +y 左），
// 这里只需按当前 yaw 转到 map 系、乘个增益、限幅，发成新的目标点。
// 增益 <1 是因为位置环本身有超调；限幅是防止一次误检把飞机拽飞。
//
// 返回 true = 像素误差已进死区（对中完成）。
bool FireMissionNode::servoStep(double z_dm, const char * tag)
{
  const rclcpp::Time now = this->now();

  // 目标丢失：保持上一个目标点不动，等它回来。超时由调用方的 servo_timeout_sec_ 兜底。
  if (!has_servo_error_ ||
      (now - servo_error_time_).seconds() > servo_data_timeout_sec_)
  {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "视觉伺服：暂时丢失目标，保持悬停");
    return false;
  }

  if (std::hypot(servo_ex_, servo_ey_) <= servo_deadband_ratio_) return true;

  double x_dm, y_dm, yaw_deg;
  if (!getCurrentPose(x_dm, y_dm, yaw_deg)) return false;

  // 机体 → map（cm），限幅后加到当前位姿上
  const double yaw = yaw_deg * M_PI / 180.0;
  double dx_cm = servo_gain_ * (std::cos(yaw) * servo_bx_cm_ - std::sin(yaw) * servo_by_cm_);
  double dy_cm = servo_gain_ * (std::sin(yaw) * servo_bx_cm_ + std::cos(yaw) * servo_by_cm_);
  const double step_dm = cmToDm(std::hypot(dx_cm, dy_cm));
  if (step_dm > servo_max_step_dm_) {
    const double scale = servo_max_step_dm_ / step_dm;
    dx_cm *= scale;
    dy_cm *= scale;
  }

  double cur_x_cm, cur_y_cm;
  fieldToMap(x_dm, y_dm, cur_x_cm, cur_y_cm);
  double tgt_x_dm, tgt_y_dm;
  mapToField(cur_x_cm + dx_cm, cur_y_cm + dy_cm, tgt_x_dm, tgt_y_dm);

  fire_wp_ = FireWaypoint{tgt_x_dm, tgt_y_dm, z_dm, patrol_yaw_deg_, -1, tag};
  publishTarget(fire_wp_);
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
    "视觉伺服[%s]：像素误差(%.3f, %.3f) → 目标 (%.1f, %.1f) dm",
    tag, servo_ex_, servo_ey_, tgt_x_dm, tgt_y_dm);
  return false;
}

// ─── 坐标变换 ───────────────────────────────────────────────────────────────
//
// map 原点 = Cartographer 上电位姿 = 起飞点。正常摆放时机头朝场地 +x，两系只差平移；
// 若现场只能斜着摆，field_yaw_offset_deg_ 给出场地 +x 在 map 系下的方位角。
void FireMissionNode::fieldToMap(double x_dm, double y_dm, double & x_cm, double & y_cm) const
{
  const double dx = dmToCm(x_dm - home_field_x_dm_);
  const double dy = dmToCm(y_dm - home_field_y_dm_);
  const double c = std::cos(field_yaw_offset_deg_ * M_PI / 180.0);
  const double s = std::sin(field_yaw_offset_deg_ * M_PI / 180.0);
  x_cm = c * dx - s * dy;
  y_cm = s * dx + c * dy;
}

void FireMissionNode::mapToField(double x_cm, double y_cm, double & x_dm, double & y_dm) const
{
  const double c = std::cos(field_yaw_offset_deg_ * M_PI / 180.0);
  const double s = std::sin(field_yaw_offset_deg_ * M_PI / 180.0);
  const double dx =  c * x_cm + s * y_cm;
  const double dy = -s * x_cm + c * y_cm;
  x_dm = cmToDm(dx) + home_field_x_dm_;
  y_dm = cmToDm(dy) + home_field_y_dm_;
}

// ─── 航点构建 ───────────────────────────────────────────────────────────────
//
// 蛇形全覆盖：每条带只要两个端点航点（激光垂直向下，飞过即扫过）。
// 起飞点在左下，第一条带 y=4 与起飞点最近，顺路；最后一条带 y=36 结束后斜插回 home。
// 巡逻里程 = 5×40 + 4×8 = 232dm；条带数为奇数，跑完停在右上 (44,36)，返航腿约 52dm。
// （竖着飞 6 条带里程同为 232dm、返航仅 40dm，但多一次转弯，实测转弯不慢再考虑换。）
void FireMissionNode::buildPatrolWaypoints()
{
  waypoints_.clear();

  const double z = patrol_height_dm_;

  // 1) 起飞：原地升到巡逻高度
  waypoints_.push_back({home_field_x_dm_, home_field_y_dm_, z, patrol_yaw_deg_, -1, "takeoff"});

  // 2) 蛇形条带
  for (std::size_t i = 0; i < strip_y_dm_.size(); ++i) {
    const double y = strip_y_dm_[i];
    const bool forward = (i % 2 == 0);           // 偶数条带 +x，奇数条带 -x
    const double x_enter = forward ? patrol_x_min_dm_ : patrol_x_max_dm_;
    const double x_exit  = forward ? patrol_x_max_dm_ : patrol_x_min_dm_;
    waypoints_.push_back({x_enter, y, z, patrol_yaw_deg_, static_cast<int>(i), "strip_enter"});
    waypoints_.push_back({x_exit,  y, z, patrol_yaw_deg_, static_cast<int>(i), "strip_exit"});
  }

  // 3) 返航 + 降落
  waypoints_.push_back({home_field_x_dm_, home_field_y_dm_, z, patrol_yaw_deg_, -1, "return"});
  waypoints_.push_back({home_field_x_dm_, home_field_y_dm_, land_height_dm_, patrol_yaw_deg_, -1, "land"});
}

FireWaypoint FireMissionNode::fireWaypoint(double z_dm, const char * tag) const
{
  return FireWaypoint{fire_x_dm_, fire_y_dm_, z_dm, patrol_yaw_deg_, -1, tag};
}

// ─── 发布 ─────────────────────────────────────────────────────────────────
void FireMissionNode::publishTarget(const FireWaypoint & wp)
{
  double x_cm, y_cm;
  fieldToMap(wp.x_dm, wp.y_dm, x_cm, y_cm);

  std_msgs::msg::Float32MultiArray msg;
  msg.data = {static_cast<float>(x_cm), static_cast<float>(y_cm),
              static_cast<float>(dmToCm(wp.z_dm)), static_cast<float>(wp.yaw_deg)};
  target_pub_->publish(msg);

  std_msgs::msg::UInt8 active_msg;
  active_msg.data = 2;
  active_controller_pub_->publish(active_msg);

  // 打开 uart_to_stm32 的速度转发门；反复发无副作用，可覆盖 uart 节点重连。
  std_msgs::msg::UInt8 route_msg;
  route_msg.data = 1;
  route_choice_pub_->publish(route_msg);
}

void FireMissionNode::publishLaser(bool on)
{
  // magnet_control_node 的命令字：1=on, 2=off（不是 1/0）。
  std_msgs::msg::Int32 msg;
  msg.data = on ? 1 : 2;
  laser_pub_->publish(msg);
}

void FireMissionNode::publishAlarmLed(bool on)
{
  std_msgs::msg::UInt8 msg;
  msg.data = on ? 1 : 0;
  alarm_led_pub_->publish(msg);
}

void FireMissionNode::publishDrop(bool on)
{
  std_msgs::msg::UInt8 msg;
  msg.data = on ? 1 : 0;
  drop_pub_->publish(msg);
  RCLCPP_INFO(get_logger(), "/drop_package = %u", static_cast<unsigned>(msg.data));
}

// 只发一条话题。UDP 丢包冗余由 fire_link_node 负责（同一 seq 连发 report_repeat 次），
// 这里循环发会变成多个不同 seq 的独立火源事件——包量翻倍、与契约文档不符。
void FireMissionNode::publishFireReport(const char * reason)
{
  std_msgs::msg::Float32MultiArray report;
  report.data = {static_cast<float>(fire_x_dm_), static_cast<float>(fire_y_dm_)};
  fire_report_pub_->publish(report);
  fire_reported_ = true;
  RCLCPP_INFO(get_logger(), "[%s] 上报火源坐标 (%.1f, %.1f) dm → 消防车",
    reason, fire_x_dm_, fire_y_dm_);
}

// ─── 遥测：1Hz 位置 + 累计里程 → 消防车 ─────────────────────────────────
void FireMissionNode::telemetryTimerCallback()
{
  std::lock_guard<std::mutex> lock(mutex_);

  double x_dm, y_dm, yaw;
  if (!getCurrentPose(x_dm, y_dm, yaw)) return;

  std_msgs::msg::Float32MultiArray pose_msg;
  pose_msg.data = {static_cast<float>(x_dm), static_cast<float>(y_dm)};
  drone_pose_pub_->publish(pose_msg);

  std_msgs::msg::Float32MultiArray dist_msg;
  dist_msg.data = {static_cast<float>(patrol_distance_dm_)};
  patrol_distance_pub_->publish(dist_msg);
}

// ─── 状态心跳 ───────────────────────────────────────────────────────────
std::string FireMissionNode::buildStatusText() const
{
  const char * phase_name = "?";
  switch (phase_) {
    case MissionPhase::WAIT_START: phase_name = "待命(等消防车按键)"; break;
    case MissionPhase::TAKEOFF:    phase_name = "起飞"; break;
    case MissionPhase::PATROL:     phase_name = "巡逻"; break;
    case MissionPhase::APPROACH:   phase_name = "接近火源"; break;
    case MissionPhase::DESCEND:    phase_name = "降高"; break;
    case MissionPhase::HOVER:      phase_name = "悬停"; break;
    case MissionPhase::DROP:       phase_name = "抛包"; break;
    // SERVO_FINE 沿用"接近火源"这个名字：fire_link_pkg::parsePhase 和车端仪表盘的
    // PHASES 字典都是按这套中文名匹配的，新增名字会显示成"未知"。
    case MissionPhase::SERVO_FINE: phase_name = "接近火源"; break;
    case MissionPhase::RESUME:     phase_name = "恢复巡逻"; break;
    case MissionPhase::RETURN:     phase_name = "返航"; break;
    case MissionPhase::LAND:       phase_name = "降落"; break;
    case MissionPhase::DONE:       phase_name = "完成"; break;
  }

  std::ostringstream os;
  os << "阶段=" << phase_name
     << ",航点=" << current_idx_ << "/" << waypoints_.size()
     << ",高度=" << static_cast<int>(current_height_dm_) << "dm"
     << ",里程=" << static_cast<int>(patrol_distance_dm_) << "dm"
     << ",视觉=" << (latest_fire_detected_ ? "见红" : "无");
  if (fire_confirmed_) {
    os << ",火源=(" << static_cast<int>(fire_x_dm_) << "," << static_cast<int>(fire_y_dm_) << ")";
    os << (fire_reported_ ? ",已报坐标" : ",未报坐标");
    os << (fire_handled_ ? ",已抛包" : ",未抛包");
  }
  return os.str();
}

void FireMissionNode::publishStatus()
{
  const std::string text = buildStatusText();
  const rclcpp::Time now = this->now();
  // 内容变了立刻发，否则每 ~0.5s 发一次充当心跳（断流即判飞机离线）
  if (status_inited_ && text == last_status_text_ &&
      (now - last_status_pub_time_).seconds() < 0.5)
  {
    return;
  }
  std_msgs::msg::String msg;
  msg.data = text;
  fire_status_pub_->publish(msg);
  last_status_text_ = text;
  last_status_pub_time_ = now;
  status_inited_ = true;
}

// ─── 位姿 / 到达判定 ────────────────────────────────────────────────────
bool FireMissionNode::getCurrentPose(double & x_dm, double & y_dm, double & yaw_deg)
{
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "无法获取 %s → %s 的 tf：%s", map_frame_.c_str(), base_frame_.c_str(), e.what());
    return false;
  }
  mapToField(tf.transform.translation.x * 100.0, tf.transform.translation.y * 100.0, x_dm, y_dm);
  yaw_deg = normalizeAngleDeg(tf2::getYaw(tf.transform.rotation) * 180.0 / M_PI);
  return true;
}

bool FireMissionNode::isReached(const FireWaypoint & wp, double x_dm, double y_dm,
                                double z_dm, double yaw_deg) const
{
  const double dxy  = std::hypot(wp.x_dm - x_dm, wp.y_dm - y_dm);
  const double dz   = std::fabs(wp.z_dm - z_dm);
  const double dyaw = std::fabs(normalizeAngleDeg(wp.yaw_deg - yaw_deg));
  return dxy <= pos_tol_dm_ && dz <= height_tol_dm_ && dyaw <= yaw_tol_deg_;
}

void FireMissionNode::accumulateDistance(double x_dm, double y_dm)
{
  if (!has_last_sample_) {
    last_sample_x_dm_ = x_dm;
    last_sample_y_dm_ = y_dm;
    has_last_sample_ = true;
    return;
  }
  const double d = std::hypot(x_dm - last_sample_x_dm_, y_dm - last_sample_y_dm_);
  if (d < dist_sample_min_dm_) return;   // 位姿抖动不计入里程
  patrol_distance_dm_ += d;
  last_sample_x_dm_ = x_dm;
  last_sample_y_dm_ = y_dm;
}

// ─── 火情处理入口 ───────────────────────────────────────────────────────
void FireMissionNode::enterFireHandling()
{
  patrol_resume_idx_ = current_idx_;
  // 起手先停在当前位置（保持巡逻高度），随后由 APPROACH 的视觉伺服一步步修到火源正上方。
  // 不再直接飞反投影坐标——那个值只用来触发本流程。
  double x_now, y_now, yaw_now;
  if (getCurrentPose(x_now, y_now, yaw_now)) {
    fire_wp_ = FireWaypoint{x_now, y_now, patrol_height_dm_, patrol_yaw_deg_, -1, "fire_hold"};
  } else {
    fire_wp_ = fireWaypoint(patrol_height_dm_, "fire_above");
  }
  use_fire_wp_ = true;
  phase_ = MissionPhase::APPROACH;
  servo_timed_out_ = false;
  phase_start_time_ = this->now();
  publishAlarmLed(true);   // 发挥(1)：识别确认后机上 LED 指示灯示警
  publishTarget(fire_wp_);
  RCLCPP_INFO(get_logger(),
    "进入火情处理：火源 (%.1f, %.1f) dm，巡逻航点 %zu 处挂起",
    fire_x_dm_, fire_y_dm_, patrol_resume_idx_);
}

// ─── 推进 ───────────────────────────────────────────────────────────────
void FireMissionNode::advance()
{
  if (current_idx_ + 1 >= waypoints_.size()) {
    // 最后一个航点（land）已到达
    phase_ = MissionPhase::DONE;
    publishLaser(false);
    publishAlarmLed(false);
    if (!mission_complete_sent_) {
      mission_complete_pub_->publish(std_msgs::msg::Empty());
      mission_complete_sent_ = true;
    }
    std_msgs::msg::UInt8 stop_msg;
    stop_msg.data = 0;
    active_controller_pub_->publish(stop_msg);
    RCLCPP_INFO(get_logger(), "任务完成：累计巡逻里程 %.1f dm", patrol_distance_dm_);
    return;
  }

  ++current_idx_;
  const FireWaypoint & wp = waypoints_[current_idx_];

  // 阶段随航点 tag 迁移（火情处理期间不改 phase_，由状态机自己管）
  if (std::string(wp.tag) == "return") {
    phase_ = MissionPhase::RETURN;
  } else if (std::string(wp.tag) == "land") {
    phase_ = MissionPhase::LAND;
  } else {
    phase_ = MissionPhase::PATROL;
  }
  phase_start_time_ = this->now();
  publishTarget(wp);
  RCLCPP_INFO(get_logger(), "→ 航点 %zu [%s] (%.1f, %.1f, %.1f) dm",
    current_idx_, wp.tag, wp.x_dm, wp.y_dm, wp.z_dm);
}

// ─── 主循环 ─────────────────────────────────────────────────────────────
void FireMissionNode::monitorTimerCallback()
{
  std::lock_guard<std::mutex> lock(mutex_);
  publishStatus();

  if (phase_ == MissionPhase::DONE) return;
  if (phase_ == MissionPhase::WAIT_START) return;   // 不接管控制器，等按键
  if (!has_height_) return;

  double x, y, yaw;
  if (!getCurrentPose(x, y, yaw)) return;
  accumulateDistance(x, y);

  if (!first_publish_done_) {
    publishLaser(true);   // 起飞即点亮机腹激光笔，全程指示航迹
    publishTarget(waypoints_[current_idx_]);
    first_publish_done_ = true;
    phase_ = MissionPhase::TAKEOFF;
    return;
  }

  const rclcpp::Time now = this->now();

  // ── 火情处理子状态机（发挥部分）──
  if (use_fire_wp_) {
    const bool at_fire = isReached(fire_wp_, x, y, current_height_dm_, yaw);
    switch (phase_) {
      case MissionPhase::APPROACH: {
        // 巡航高度粗对中。超时兜底：不再等对中，照常降高抛包（发挥2 尽量拿分）。
        const bool timed_out =
          (now - phase_start_time_).seconds() >= servo_timeout_sec_;
        if (timed_out && !servo_timed_out_) {
          servo_timed_out_ = true;
          RCLCPP_WARN(get_logger(), "粗对中超时 %.0fs，放弃对中直接降高抛包", servo_timeout_sec_);
        }
        if (!servoStep(patrol_height_dm_, "fire_servo_coarse") && !timed_out) return;

        // 粗对中完成（或超时）→ 此刻飞机就在火源正上方，直接用**自身位姿**当火源坐标
        // 上报给消防车，比反投影准得多，也早于降高/抛包，车能尽快出发。
        double x_now, y_now, yaw_now;
        if (getCurrentPose(x_now, y_now, yaw_now)) {
          fire_x_dm_ = x_now;
          fire_y_dm_ = y_now;
        }
        publishFireReport(timed_out ? "粗对中超时(用当前位姿)" : "粗对中完成");

        fire_wp_ = fireWaypoint(drop_height_dm_, "fire_drop_height");
        phase_ = MissionPhase::DESCEND;
        phase_start_time_ = now;
        publishTarget(fire_wp_);
        return;
      }

      case MissionPhase::DESCEND:
        if (!at_fire) return;
        phase_ = MissionPhase::SERVO_FINE;
        phase_start_time_ = now;
        servo_timed_out_ = false;
        return;

      case MissionPhase::SERVO_FINE: {
        // 10dm 高度再对一次：目标在画面里大了近一倍，同样 5% 死区对应的地面误差减半。
        const bool timed_out =
          (now - phase_start_time_).seconds() >= servo_timeout_sec_;
        if (!servoStep(drop_height_dm_, "fire_servo_fine") && !timed_out) return;
        if (timed_out) {
          RCLCPP_WARN(get_logger(), "精对中超时 %.0fs，直接进入悬停抛包", servo_timeout_sec_);
        }
        phase_ = MissionPhase::HOVER;
        phase_start_time_ = now;
        return;
      }

      case MissionPhase::HOVER:
        // 题目要求悬停 3s 后再抛
        if ((now - phase_start_time_).seconds() < hover_before_drop_sec_) return;
        phase_ = MissionPhase::DROP;
        phase_start_time_ = now;
        drop_sent_ = false;
        return;

      case MissionPhase::DROP:
        if (!drop_sent_) {
          publishDrop(true);
          drop_sent_ = true;
          drop_sent_time_ = now;
          return;
        }
        if ((now - drop_sent_time_).seconds() < drop_pulse_sec_) return;
        publishDrop(false);
        // 抛完再报一次（发挥2）。粗对中时已经报过一次，这次是补一刀：
        // 万一那次的 5 个 UDP 包全丢了，这里还有一次机会。
        // 此时已经过精对中，位姿比粗对中那次更准，用当前值覆盖。
        {
          double x_drop, y_drop, yaw_drop;
          if (getCurrentPose(x_drop, y_drop, yaw_drop)) {
            fire_x_dm_ = x_drop;
            fire_y_dm_ = y_drop;
          }
        }
        publishFireReport("抛包完成");
        fire_handled_ = true;
        publishAlarmLed(false);
        // 拉回巡逻高度
        fire_wp_ = fireWaypoint(patrol_height_dm_, "fire_climb");
        phase_ = MissionPhase::RESUME;
        phase_start_time_ = now;
        publishTarget(fire_wp_);
        return;

      case MissionPhase::RESUME: {
        if (!at_fire) return;
        // 归位分两步，**不沿航向回退**：
        //   ① 已爬回巡逻高度（fire_climb 到达即本分支）
        //   ② 只修横向（场地 y，即条带间距那个方向）回到被打断条带的中心线，
        //      场地 x 保持当前值 —— 火源处理期间在航向上的位移不用倒回去。
        // 然后交回主航线，直接朝原方向飞到该条带的端点。
        const double strip_y_dm = waypoints_[patrol_resume_idx_].y_dm;
        if (std::fabs(y - strip_y_dm) > pos_tol_dm_) {
          fire_wp_ = FireWaypoint{x, strip_y_dm, patrol_height_dm_,
                                  patrol_yaw_deg_, -1, "fire_rejoin"};
          publishTarget(fire_wp_);
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "归位：横向回到条带中心线 y=%.1f dm（x 保持 %.1f，不回退）", strip_y_dm, x);
          return;
        }
        use_fire_wp_ = false;
        current_idx_ = patrol_resume_idx_;
        phase_ = MissionPhase::PATROL;
        phase_start_time_ = now;
        publishTarget(waypoints_[current_idx_]);
        RCLCPP_INFO(get_logger(), "火情处理完毕，回到巡逻航点 %zu 继续", current_idx_);
        return;
      }

      default:
        return;
    }
  }

  // ── 巡逻中发现火情 → 打断 ──
  if (enable_fire_task_ && !fire_handled_ && fire_confirmed_ &&
      phase_ == MissionPhase::PATROL)
  {
    enterFireHandling();
    return;
  }

  // ── 常规航点推进 ──
  const FireWaypoint & wp = waypoints_[current_idx_];
  if (!isReached(wp, x, y, current_height_dm_, yaw)) return;

  advance();
}

// ─── 工具函数 ───────────────────────────────────────────────────────────
double FireMissionNode::normalizeAngleDeg(double angle_deg) const
{
  while (angle_deg > 180.0)  angle_deg -= 360.0;
  while (angle_deg < -180.0) angle_deg += 360.0;
  return angle_deg;
}

}  // namespace fire_control_pkg
