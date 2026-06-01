#include "inventory_control_pkg/inventory_mission_node.hpp"

#include <algorithm>
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
  if (target_identified_) {
    return true;
  }
  publishQrEnable(true);  // 起飞前开识别，对着手持的抽取码

  bool has_id; std::string id;
  { std::lock_guard<std::mutex> lk(mutex_); has_id = has_qr_id_; id = latest_qr_id_; }
  if (!has_id) {
    return false;
  }

  target_cargo_id_ = parseCargoId(id);
  // TODO(标定)：编号→货位 的映射在比赛时未知（二维码随机张贴），
  // 题目要求2 只要求"飞到目标货物"。可行做法：
  //   1) 先飞要求1 的遍历建好 inventory_ 表（编号→货位），DIRECTED 时查表得 slot；
  //   2) 或 DIRECTED 单独飞一条"边飞边读"的航线，读到匹配编号即盘点降落。
  // 这里先用查表，查不到则退化为遍历式搜索（见 buildDirectedWaypoints）。
  target_slot_.clear();
  for (const auto & kv : inventory_) {
    if (kv.second == target_cargo_id_) { target_slot_ = kv.first; break; }
  }

  std_msgs::msg::String t;
  t.data = "目标编号=" + std::to_string(target_cargo_id_) +
           (target_slot_.empty() ? ",货位=未知(搜索)" : ",货位=" + target_slot_);
  inventory_target_pub_->publish(t);

  publishQrEnable(false);
  target_identified_ = true;
  RCLCPP_INFO(get_logger(), "抽取码识别完成：%s", t.data.c_str());
  return true;
}

// ───────────────────────────── 航线构建
void InventoryMissionNode::buildTraverseWaypoints()
{
  waypoints_.clear();

  // 起飞航点（在 home 上空升到巡航高）
  waypoints_.push_back({home_x_cm_, home_y_cm_, flight_height_cm_, 0.0,
                        false, "", "takeoff"});

  // TODO(场地标定)：以下坐标全部为占位示意，需按本机 cartographer 地图坐标系
  // 与题目 图1/图2 实测标定。当前仅给出"单相机 + yaw 换面"的航点骨架结构：
  //   - 每个面 6 个货位 = 2 行 × 3 列；上行高 ~125cm、下行高 ~25cm（图2：40+40 间距，下沿60）
  //   - A 面与 B 面在同一货架两侧，yaw 相差 180°；C/D 面同理在货架2。
  //   - scan_standoff_cm_ 为相机离板面的水平后撤距离。
  struct FaceDef { const char * face; double shelf_x; double yaw; double standoff_sign; };
  const std::array<FaceDef, 4> faces = {{
    {"A", shelf1_x_cm_, 0.0,   -1.0},
    {"B", shelf1_x_cm_, 180.0, +1.0},
    {"C", shelf2_x_cm_, 0.0,   -1.0},
    {"D", shelf2_x_cm_, 180.0, +1.0},
  }};

  const double y_center = (home_y_cm_ + 250.0);  // 占位：货架沿 y 方向中心
  const std::array<double, 2> row_z = {flight_height_cm_, flight_height_cm_ - slot_row_spacing_cm_};
  const std::array<double, 3> col_dy = {-slot_col_spacing_cm_, 0.0, slot_col_spacing_cm_};

  for (const auto & f : faces) {
    const double face_x = f.shelf_x + f.standoff_sign * scan_standoff_cm_;
    for (int row = 0; row < 2; ++row) {       // 0=上行(1,2,3) 1=下行(4,5,6)
      for (int col = 0; col < 3; ++col) {
        const int idx = row * 3 + col + 1;     // 1..6
        const std::string slot = std::string(f.face) + std::to_string(idx);
        waypoints_.push_back({face_x, y_center + col_dy[col], row_z[row], f.yaw,
                              true, slot, "scan"});
      }
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

  if (!target_slot.empty()) {
    // TODO(场地标定)：由 target_slot 查面/行/列反算 (x,y,z,yaw)，直飞该货位。
    // 占位：直接飞到货架1 A 面中心高度悬停盘点。
    const double face_x = shelf1_x_cm_ - scan_standoff_cm_;
    const double y_center = home_y_cm_ + 250.0;
    waypoints_.push_back({face_x, y_center, flight_height_cm_, 0.0,
                          true, target_slot, "scan_directed"});
  } else {
    RCLCPP_WARN(get_logger(),
      "目标货位未知：需先跑遍历建表，或改用边飞边读搜索航线（见 cpp TODO）。");
    // 占位：退化为遍历搜索
    buildTraverseWaypoints();
    return;
  }

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
