#include "inventory_control_pkg/inventory_mission_node.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace inventory_control_pkg
{

InventoryMissionNode::InventoryMissionNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("inventory_mission_node", options),
  active_mode_(MissionMode::TRAVERSE),
  phase_(MissionPhase::TAKEOFF),
  current_idx_(0),
  scan_active_(false),
  scan_qr_aligned_(false),
  has_qr_id_(false),
  target_identified_(false),
  target_cargo_id_(0),
  has_target_slot_(false),
  has_height_(false),
  current_height_cm_(0.0),
  mission_complete_sent_(false),
  first_publish_done_(false)
{
  // ── 参数 ──
  map_frame_  = declare_parameter<std::string>("map_frame", "map");
  base_frame_ = declare_parameter<std::string>("base_frame", "laser_link");

  mode_str_ = declare_parameter<std::string>("mode", "traverse");
  mode_ = (mode_str_ == "directed") ? MissionMode::DIRECTED : MissionMode::TRAVERSE;
  active_mode_ = mode_;

  pos_tol_cm_    = declare_parameter<double>("pos_tol_cm", 12.0);
  yaw_tol_deg_   = declare_parameter<double>("yaw_tol_deg", 8.0);
  height_tol_cm_ = declare_parameter<double>("height_tol_cm", 15.0);

  flight_height_cm_ = declare_parameter<double>("flight_height_cm", 150.0);
  land_height_cm_   = declare_parameter<double>("land_height_cm", 0.0);
  home_x_cm_ = declare_parameter<double>("home_x_cm", 0.0);
  home_y_cm_ = declare_parameter<double>("home_y_cm", 0.0);
  land_x_cm_ = declare_parameter<double>("land_x_cm", 0.0);
  land_y_cm_ = declare_parameter<double>("land_y_cm", 0.0);

  scan_settle_sec_  = declare_parameter<double>("scan_settle_sec", 0.8);
  scan_timeout_sec_ = declare_parameter<double>("scan_timeout_sec", 6.0);
  led_blink_sec_    = declare_parameter<double>("led_blink_sec", 1.0);

  shelf1_x_cm_         = declare_parameter<double>("shelf1_x_cm", 150.0);
  shelf2_x_cm_         = declare_parameter<double>("shelf2_x_cm", 350.0);
  slot_col_spacing_cm_ = declare_parameter<double>("slot_col_spacing_cm", 50.0);
  slot_row_spacing_cm_ = declare_parameter<double>("slot_row_spacing_cm", 80.0);
  scan_standoff_cm_    = declare_parameter<double>("scan_standoff_cm", 60.0);

  // ── 发布 ──
  target_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("/target_position", 10);
  active_controller_pub_ = create_publisher<std_msgs::msg::UInt8>("/active_controller", 10);
  route_choice_pub_ = create_publisher<std_msgs::msg::UInt8>("/route_choice", 10);
  qr_enable_pub_ = create_publisher<std_msgs::msg::Bool>("/qr_vision/enable", 10);
  inventory_result_pub_ = create_publisher<std_msgs::msg::String>("/inventory_result", 10);
  inventory_led_pub_ = create_publisher<std_msgs::msg::Empty>("/inventory_led", 10);
  inventory_target_pub_ = create_publisher<std_msgs::msg::String>("/inventory_target", 10);
  mission_complete_pub_ = create_publisher<std_msgs::msg::Empty>("/mission_complete", 10);

  // ── 订阅 ──
  height_sub_ = create_subscription<std_msgs::msg::Int16>(
    "/height", 10,
    std::bind(&InventoryMissionNode::heightCallback, this, std::placeholders::_1));
  qr_id_sub_ = create_subscription<std_msgs::msg::String>(
    "/qr_vision/id", 10,
    std::bind(&InventoryMissionNode::qrIdCallback, this, std::placeholders::_1));
  qr_aligned_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/qr_vision/aligned", 10,
    std::bind(&InventoryMissionNode::qrAlignedCallback, this, std::placeholders::_1));
  // 要求2：地面站查表后下发目标货位（"A1".."D6"），飞机据此直飞
  target_slot_sub_ = create_subscription<std_msgs::msg::String>(
    "/inventory_target_slot", 10,
    std::bind(&InventoryMissionNode::targetSlotCallback, this, std::placeholders::_1));

  // ── tf ──
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // ── 航线 ──
  if (active_mode_ == MissionMode::TRAVERSE) {
    buildTraverseWaypoints();
    phase_ = MissionPhase::TAKEOFF;
  } else {
    // DIRECTED：先在地面识别抽取码，识别成功后再 buildDirectedWaypoints。
    phase_ = MissionPhase::IDENTIFY;
  }

  // 平时关识别（避免误打激光）
  publishQrEnable(false);

  monitor_timer_ = create_wall_timer(
    std::chrono::milliseconds(50),
    std::bind(&InventoryMissionNode::monitorTimerCallback, this));

  RCLCPP_INFO(get_logger(), "InventoryMissionNode 启动，mode=%s，航点数=%zu",
              mode_str_.c_str(), waypoints_.size());
}

// ───────────────────────────── 订阅回调
void InventoryMissionNode::heightCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mutex_);
  has_height_ = true;
  current_height_cm_ = static_cast<double>(msg->data);
}

void InventoryMissionNode::qrIdCallback(const std_msgs::msg::String::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mutex_);
  latest_qr_id_ = msg->data;
  has_qr_id_ = true;
}

void InventoryMissionNode::qrAlignedCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(mutex_);
  scan_qr_aligned_ = msg->data;
}

void InventoryMissionNode::targetSlotCallback(const std_msgs::msg::String::SharedPtr msg)
{
  // 地面站下发的货位字符串（"A1".."D6"）。只取首字母+数字，容忍前后空白。
  std::string s;
  for (char c : msg->data) {
    if (!std::isspace(static_cast<unsigned char>(c))) s += c;
  }
  if (s.empty()) {
    return;
  }
  std::lock_guard<std::mutex> lk(mutex_);
  target_slot_ = s;
  has_target_slot_ = true;
}

// ───────────────────────────── 主循环
void InventoryMissionNode::monitorTimerCallback()
{
  // 头一拍先把控制器使能 + uart 开门发出去
  if (!first_publish_done_) {
    std_msgs::msg::UInt8 ctl; ctl.data = 2;          // 位置控制器接管
    active_controller_pub_->publish(ctl);
    std_msgs::msg::UInt8 rc; rc.data = 1;            // uart 开门转发速度
    route_choice_pub_->publish(rc);
    first_publish_done_ = true;
  }

  switch (phase_) {
    case MissionPhase::IDENTIFY: {
      if (runIdentifyTarget()) {
        buildDirectedWaypoints(target_slot_);
        current_idx_ = 0;
        phase_ = MissionPhase::TAKEOFF;
      }
      return;
    }
    case MissionPhase::DONE: {
      if (!mission_complete_sent_) {
        std_msgs::msg::Empty e; mission_complete_pub_->publish(e);
        std_msgs::msg::UInt8 ctl; ctl.data = 0;
        active_controller_pub_->publish(ctl);
        publishQrEnable(false);
        mission_complete_sent_ = true;
        RCLCPP_INFO(get_logger(), "盘点任务完成，共记录 %zu 个货物。", inventory_.size());
      }
      return;
    }
    default: break;
  }

  if (current_idx_ >= waypoints_.size()) {
    phase_ = MissionPhase::DONE;
    return;
  }

  const InventoryWaypoint & wp = waypoints_[current_idx_];
  publishTarget(wp);

  double x_cm, y_cm, yaw_deg;
  if (!getCurrentPose(x_cm, y_cm, yaw_deg)) {
    return;  // 还没拿到位姿，先持续发目标
  }
  double z_cm;
  { std::lock_guard<std::mutex> lk(mutex_); z_cm = current_height_cm_; }

  if (!isReached(wp, x_cm, y_cm, z_cm, yaw_deg)) {
    return;  // 未到点，继续逼近
  }

  // 到点。盘点航点要跑盘点子状态；过渡航点直接推进。
  if (wp.scan) {
    if (!scan_active_) {
      scan_active_ = true;
      scan_start_time_ = now();
      scan_qr_aligned_ = false;
      has_qr_id_ = false;
      publishQrEnable(true);   // 开识别+激光
    }
    if (runScanAtWaypoint()) {
      publishQrEnable(false);
      scan_active_ = false;
      advance();
    }
  } else {
    advance();
  }
}

// ───────────────────────────── 盘点子状态
bool InventoryMissionNode::runScanAtWaypoint()
{
  const double elapsed = (now() - scan_start_time_).seconds();
  if (elapsed < scan_settle_sec_) {
    return false;  // 先稳一下再判定
  }

  bool aligned, has_id; std::string id;
  { std::lock_guard<std::mutex> lk(mutex_);
    aligned = scan_qr_aligned_; has_id = has_qr_id_; id = latest_qr_id_; }

  if (aligned && has_id) {
    const int cargo = parseCargoId(id);
    const std::string & slot = waypoints_[current_idx_].slot;
    recordInventory(slot, cargo);

    std_msgs::msg::String res;
    res.data = "编号=" + std::to_string(cargo) + ",货位=" + slot;
    inventory_result_pub_->publish(res);

    std_msgs::msg::Empty led; inventory_led_pub_->publish(led);  // 地面站 LED 亮灭一次

    RCLCPP_INFO(get_logger(), "盘点 %s -> 货物编号 %d", slot.c_str(), cargo);
    return true;
  }

  if (elapsed > scan_timeout_sec_) {
    RCLCPP_WARN(get_logger(), "货位 %s 盘点超时，跳过。",
                waypoints_[current_idx_].slot.c_str());
    return true;  // 超时也推进，避免整套卡死
  }
  return false;
}

// ───────────────────────────── DIRECTED：地面识别抽取码
bool InventoryMissionNode::runIdentifyTarget()
{
  // 第一步：地面识别"抽取的那张码"，拿到编号后报送地面站（要求2-1）。
  if (!target_identified_) {
    publishQrEnable(true);  // 起飞前开识别，对着手持的抽取码

    bool has_id; std::string id;
    { std::lock_guard<std::mutex> lk(mutex_); has_id = has_qr_id_; id = latest_qr_id_; }
    if (!has_id) {
      return false;
    }

    target_cargo_id_ = parseCargoId(id);
    std_msgs::msg::String t;
    t.data = std::to_string(target_cargo_id_);   // 报送编号给地面站
    inventory_target_pub_->publish(t);

    publishQrEnable(false);
    target_identified_ = true;
    RCLCPP_INFO(get_logger(),
      "抽取码编号 %d 已报送地面站，等待地面站查表下发货位…", target_cargo_id_);
    return false;
  }

  // 第二步：等地面站把"编号→货位"查好后下发货位（地面站权威，二维码位置固定，
  // 飞机只需货位字符串，细微偏差由相机微调）。收到货位即可规划直飞航线。
  bool ready; std::string slot;
  { std::lock_guard<std::mutex> lk(mutex_); ready = has_target_slot_; slot = target_slot_; }
  if (!ready) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
      "已报送编号 %d，等待地面站下发目标货位（/inventory_target_slot）…", target_cargo_id_);
    return false;
  }

  // 把"编号→货位"的最终结果再报一次给地面站（要求2-2 画航线图用）。
  std_msgs::msg::String t;
  t.data = "目标编号=" + std::to_string(target_cargo_id_) + ",货位=" + slot;
  inventory_target_pub_->publish(t);
  RCLCPP_INFO(get_logger(), "地面站下发目标货位 %s（编号 %d），规划直飞航线。",
              slot.c_str(), target_cargo_id_);
  return true;
}

// ───────────────────────────── 货位 → 航点（遍历/定向共用，保证一致）
InventoryWaypoint InventoryMissionNode::slotToScanWaypoint(const std::string & slot) const
{
  // slot = 面字符(A/B/C/D) + 货位号(1..6)，如 "C5"。
  const char face = slot.empty() ? 'A' : slot[0];
  int idx = 1;
  try { idx = std::stoi(slot.substr(1)); } catch (...) { idx = 1; }
  idx = std::max(1, std::min(6, idx));

  // 面 → 货架 x / yaw / standoff 后撤方向。A/B 在货架1，C/D 在货架2；
  // B/D 是背面，整机 yaw 转 180°，相机从货架另一侧（+x）后撤。
  const double shelf_x   = (face == 'A' || face == 'B') ? shelf1_x_cm_ : shelf2_x_cm_;
  const bool   back_face = (face == 'B' || face == 'D');
  const double yaw          = back_face ? 180.0 : 0.0;
  const double standoff_sign = back_face ? +1.0 : -1.0;
  const double face_x = shelf_x + standoff_sign * scan_standoff_cm_;

  const int row = (idx - 1) / 3;    // 0=上行(1,2,3) 1=下行(4,5,6)
  int       col = (idx - 1) % 3;    // 0,1,2 = 正面视角 左→右
  // 背面(B/D)相机朝向相反，列的左右在 map-y 上镜像翻转。
  // TODO(标定)：若实测 B/D 列方向反了，去掉这行镜像即可——这是唯一的列方向开关。
  if (back_face) col = 2 - col;

  // TODO(场地标定)：y_center 为货架沿 y 方向中心、row_z 为上下行真实高度，
  // 均需按本机 cartographer 地图原点 + 题目图2（下沿60、行间40+40）实测填写。
  const double y_center = home_y_cm_ + 250.0;  // 占位
  const std::array<double, 3> col_dy = {-slot_col_spacing_cm_, 0.0, slot_col_spacing_cm_};
  const std::array<double, 2> row_z  = {flight_height_cm_, flight_height_cm_ - slot_row_spacing_cm_};

  InventoryWaypoint wp;
  wp.x_cm    = face_x;
  wp.y_cm    = y_center + col_dy[col];
  wp.z_cm    = row_z[row];
  wp.yaw_deg = yaw;
  wp.scan    = true;
  wp.slot    = slot;
  wp.tag     = "scan";
  return wp;
}

// ───────────────────────────── 航线构建
void InventoryMissionNode::buildTraverseWaypoints()
{
  waypoints_.clear();

  // 起飞航点（在 home 上空升到巡航高）
  waypoints_.push_back({home_x_cm_, home_y_cm_, flight_height_cm_, 0.0,
                        false, "", "takeoff"});

  // 24 个货位，顺序 A1..A6 → B1..B6 → C1..C6 → D1..D6（A→B、C→D 之间的换面
  // 过渡航点：回实验室结合实测坐标插入，详见开发笔记 §三建议）。
  for (const char * face : {"A", "B", "C", "D"}) {
    for (int idx = 1; idx <= 6; ++idx) {
      waypoints_.push_back(slotToScanWaypoint(std::string(face) + std::to_string(idx)));
    }
  }

  // 返航 + 降落
  waypoints_.push_back({land_x_cm_, land_y_cm_, flight_height_cm_, 0.0, false, "", "return"});
  waypoints_.push_back({land_x_cm_, land_y_cm_, land_height_cm_, 0.0, false, "", "land"});
}

void InventoryMissionNode::buildDirectedWaypoints(const std::string & target_slot)
{
  waypoints_.clear();
  waypoints_.push_back({home_x_cm_, home_y_cm_, flight_height_cm_, 0.0,
                        false, "", "takeoff"});

  // 地面站已下发货位 → 用与遍历完全相同的几何映射到航点，直飞该货位盘点。
  // （二维码位置固定，slotToScanWaypoint 给出标称坐标，细微偏差由 qr_fine_tune 微调。）
  InventoryWaypoint scan = slotToScanWaypoint(target_slot);
  scan.tag = "scan_directed";
  waypoints_.push_back(scan);

  waypoints_.push_back({land_x_cm_, land_y_cm_, flight_height_cm_, 0.0, false, "", "return"});
  waypoints_.push_back({land_x_cm_, land_y_cm_, land_height_cm_, 0.0, false, "", "land"});
}

void InventoryMissionNode::recordInventory(const std::string & slot, int cargo_id)
{
  if (!slot.empty() && cargo_id > 0) {
    inventory_[slot] = cargo_id;
  }
}

// ───────────────────────────── 工具
void InventoryMissionNode::publishTarget(const InventoryWaypoint & wp)
{
  std_msgs::msg::Float32MultiArray msg;
  msg.data = {static_cast<float>(wp.x_cm), static_cast<float>(wp.y_cm),
              static_cast<float>(wp.z_cm), static_cast<float>(wp.yaw_deg)};
  target_pub_->publish(msg);
}

void InventoryMissionNode::publishQrEnable(bool on)
{
  std_msgs::msg::Bool msg; msg.data = on;
  qr_enable_pub_->publish(msg);
}

void InventoryMissionNode::advance()
{
  ++current_idx_;
  // 根据下一航点 tag 粗略更新 phase（仅用于日志/外部观测）
  if (current_idx_ < waypoints_.size()) {
    const char * tag = waypoints_[current_idx_].tag;
    if (std::string(tag) == "return")      phase_ = MissionPhase::RETURN;
    else if (std::string(tag) == "land")   phase_ = MissionPhase::LAND;
    else                                    phase_ = MissionPhase::TRAVERSE;
  }
}

bool InventoryMissionNode::getCurrentPose(double & x_cm, double & y_cm, double & yaw_deg)
{
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "tf %s->%s 查询失败: %s", map_frame_.c_str(),
                         base_frame_.c_str(), ex.what());
    return false;
  }
  x_cm = meterToCm(tf.transform.translation.x);
  y_cm = meterToCm(tf.transform.translation.y);
  yaw_deg = tf2::getYaw(tf.transform.rotation) * 180.0 / M_PI;
  return true;
}

bool InventoryMissionNode::isReached(const InventoryWaypoint & wp, double x_cm,
                                     double y_cm, double z_cm, double yaw_deg) const
{
  const double dxy = std::hypot(wp.x_cm - x_cm, wp.y_cm - y_cm);
  const double dz  = std::fabs(wp.z_cm - z_cm);
  const double dyaw = std::fabs(normalizeAngleDeg(wp.yaw_deg - yaw_deg));
  return dxy <= pos_tol_cm_ && dz <= height_tol_cm_ && dyaw <= yaw_tol_deg_;
}

double InventoryMissionNode::normalizeAngleDeg(double angle_deg) const
{
  while (angle_deg > 180.0) angle_deg -= 360.0;
  while (angle_deg < -180.0) angle_deg += 360.0;
  return angle_deg;
}

int InventoryMissionNode::parseCargoId(const std::string & text) const
{
  try {
    return std::stoi(text);
  } catch (...) {
    return -1;
  }
}

}  // namespace inventory_control_pkg
