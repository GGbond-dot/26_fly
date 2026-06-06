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
  scan_laser_fired_(false),
  scan_retreated_(false),
  retreat_in_progress_(false),
  target_identified_(false),
  target_cargo_id_(0),
  has_target_slot_(false),
  has_height_(false),
  current_height_cm_(0.0),
  mission_complete_sent_(false),
  first_publish_done_(false),
  status_inited_(false)
{
  // ── 参数 ──
  map_frame_  = declare_parameter<std::string>("map_frame", "map");
  base_frame_ = declare_parameter<std::string>("base_frame", "laser_link");

  mode_str_ = declare_parameter<std::string>("mode", "traverse");
  if (mode_str_ == "directed")          mode_ = MissionMode::DIRECTED;
  else if (mode_str_ == "rotate_test")  mode_ = MissionMode::ROTATE_TEST;
  else                                  mode_ = MissionMode::TRAVERSE;
  active_mode_ = mode_;

  // 遍历哪些面：默认四面全跑；只有货架1时设 "A,B" 即可只扫前后两面，不会飞向货架2。
  traverse_faces_str_ = declare_parameter<std::string>("traverse_faces", "A,B,C,D");
  {
    std::string tok;
    for (char c : traverse_faces_str_) {
      if (c == ',') {
        if (!tok.empty()) { traverse_faces_.push_back(tok); tok.clear(); }
      } else if (!std::isspace(static_cast<unsigned char>(c))) {
        tok += static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
      }
    }
    if (!tok.empty()) traverse_faces_.push_back(tok);
    if (traverse_faces_.empty()) traverse_faces_ = {"A", "B", "C", "D"};
  }

  pos_tol_cm_    = declare_parameter<double>("pos_tol_cm", 6.0);
  yaw_tol_deg_   = declare_parameter<double>("yaw_tol_deg", 8.0);
  height_tol_cm_ = declare_parameter<double>("height_tol_cm", 15.0);

  flight_height_cm_ = declare_parameter<double>("flight_height_cm", 150.0);
  land_height_cm_   = declare_parameter<double>("land_height_cm", 0.0);
  home_x_cm_ = declare_parameter<double>("home_x_cm", 0.0);
  home_y_cm_ = declare_parameter<double>("home_y_cm", 0.0);
  land_x_cm_ = declare_parameter<double>("land_x_cm", 0.0);
  land_y_cm_ = declare_parameter<double>("land_y_cm", 0.0);
  // 四面全跑(要求1)落黑圆 land_x/y；单货架测试(traverse_faces:=A,B)留 false 落末面 y 轴。
  land_at_circle_ = declare_parameter<bool>("land_at_circle", false);
  // 换面过渡航点(回y轴/平移/旋转)的 y：往后退离板面，默认 -20（不再贴 y=0）。
  transit_y_cm_ = declare_parameter<double>("transit_y_cm", -20.0);
  // 定向返航横移走廊的 y（板子远侧空旷处，默认 300）：沿本列爬到此 y 再横移到降落 x。
  directed_return_y_cm_ = declare_parameter<double>("directed_return_y_cm", 300.0);

  scan_settle_sec_  = declare_parameter<double>("scan_settle_sec", 0.8);
  scan_timeout_sec_ = declare_parameter<double>("scan_timeout_sec", 6.0);
  // 超时补救：沿机头反方向后撤这么多 cm 再试一次（离板远些好解码）。0=关闭后退、超时直接跳过。
  scan_retreat_cm_  = declare_parameter<double>("scan_retreat_cm", 10.0);
  // 后退重试时收严激光（standoff 变大→归一化窗口对应真实偏移变大，强制 strict_vertical 卡住纵向）。
  tighten_laser_on_retreat_ = declare_parameter<bool>("tighten_laser_on_retreat", true);
  // 与上一航点高度差超过此值才算“由升/降进入该货位”→ 开纵向严判（同行同高度横移不开）。
  vertical_entry_tol_cm_ = declare_parameter<double>("vertical_entry_tol_cm", 5.0);
  led_blink_sec_    = declare_parameter<double>("led_blink_sec", 1.0);

  shelf1_x_cm_         = declare_parameter<double>("shelf1_x_cm", 150.0);
  shelf2_x_cm_         = declare_parameter<double>("shelf2_x_cm", 350.0);
  slot_col_spacing_cm_ = declare_parameter<double>("slot_col_spacing_cm", 50.0);
  slot_row_spacing_cm_ = declare_parameter<double>("slot_row_spacing_cm", 80.0);
  scan_standoff_cm_    = declare_parameter<double>("scan_standoff_cm", 60.0);

  // 旋转测试航线参数（mode:=rotate_test）
  test_height_cm_     = declare_parameter<double>("test_height_cm", 100.0);
  test_forward_cm_    = declare_parameter<double>("test_forward_cm", 200.0);
  test_yaw_deg_       = declare_parameter<double>("test_yaw_deg", 180.0);
  // 默认 180=一口气连续转（±180 跳变已由 PID 角度模式根治，不再需要分步保险）。
  // 想分段观察可传更小值（如 90/45），每步到位停一下。
  test_yaw_step_deg_  = declare_parameter<double>("test_yaw_step_deg", 180.0);

  // ── 发布 ──
  target_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>("/target_position", 10);
  active_controller_pub_ = create_publisher<std_msgs::msg::UInt8>("/active_controller", 10);
  route_choice_pub_ = create_publisher<std_msgs::msg::UInt8>("/route_choice", 10);
  qr_enable_pub_ = create_publisher<std_msgs::msg::Bool>("/qr_vision/enable", 10);
  qr_strict_vertical_pub_ = create_publisher<std_msgs::msg::Bool>("/qr_vision/strict_vertical", 10);
  inventory_result_pub_ = create_publisher<std_msgs::msg::String>("/inventory_result", 10);
  inventory_led_pub_ = create_publisher<std_msgs::msg::Empty>("/inventory_led", 10);
  inventory_target_pub_ = create_publisher<std_msgs::msg::String>("/inventory_target", 10);
  mission_complete_pub_ = create_publisher<std_msgs::msg::Empty>("/mission_complete", 10);
  // 状态/心跳 → 地面站显示飞机起没起好、识别/盘点到哪步（操作员只能看地面站）。
  inventory_status_pub_ = create_publisher<std_msgs::msg::String>("/inventory_status", 10);

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
  qr_laser_fired_sub_ = create_subscription<std_msgs::msg::String>(
    "/qr_vision/laser_fired", 10,
    std::bind(&InventoryMissionNode::laserFiredCallback, this, std::placeholders::_1));
  // 要求2：地面站查表后下发目标货位（"A1".."D6"），飞机据此直飞
  target_slot_sub_ = create_subscription<std_msgs::msg::String>(
    "/inventory_target_slot", 10,
    std::bind(&InventoryMissionNode::targetSlotCallback, this, std::placeholders::_1));
  // 任务模式：地面站告诉重启后的飞机本轮跑 traverse(普通) 还是 directed(进阶)
  mode_cmd_sub_ = create_subscription<std_msgs::msg::String>(
    "/inventory_mode", 10,
    std::bind(&InventoryMissionNode::modeCommandCallback, this, std::placeholders::_1));

  // ── tf ──
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // ── 航线 ──
  // rotate_test 是纯调试航线，仍按 launch 参数立即配置起飞（不归地面站管）。
  // 比赛模式(traverse/directed)：飞机重启后**原地待命 WAIT_MODE**，不起飞、不接管控制器，
  // 等地面站发 /inventory_mode 告知本轮是普通还是进阶，收到才 configureForMode 配置起飞。
  if (active_mode_ == MissionMode::ROTATE_TEST) {
    configureForMode();
  } else {
    phase_ = MissionPhase::WAIT_MODE;
    RCLCPP_INFO(get_logger(),
      "原地待命：等地面站下发任务模式 /inventory_mode（traverse=普通 / directed=进阶）…");
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

void InventoryMissionNode::laserFiredCallback(const std_msgs::msg::String::SharedPtr msg)
{
  // qr_vision 把激光打满 0.5s 后回报刚打的码。只在本货位扫描进行中才采信
  // （平时 qr_vision 关识别不会发；扫描开始时已把 scan_laser_fired_ 复位，故无残留）。
  std::lock_guard<std::mutex> lk(mutex_);
  if (!scan_active_) return;
  scan_laser_fired_ = true;
  scan_laser_id_ = msg->data;
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

void InventoryMissionNode::modeCommandCallback(const std_msgs::msg::String::SharedPtr msg)
{
  // 地面站下发本轮任务模式。只在 WAIT_MODE（上电待命）时采信，已起飞/进行中忽略，避免误切。
  std::string s;
  for (char c : msg->data) {
    if (!std::isspace(static_cast<unsigned char>(c))) s += static_cast<char>(std::tolower(c));
  }
  MissionMode requested;
  if (s == "directed")      requested = MissionMode::DIRECTED;
  else if (s == "traverse") requested = MissionMode::TRAVERSE;
  else {
    RCLCPP_WARN(get_logger(), "收到无法识别的任务模式 '%s'（应为 traverse/directed），忽略。",
                msg->data.c_str());
    return;
  }

  if (phase_ != MissionPhase::WAIT_MODE) {
    RCLCPP_WARN(get_logger(), "已开始任务（非待命态），忽略地面站模式切换 '%s'。", s.c_str());
    return;
  }

  active_mode_ = requested;
  mode_str_ = s;
  RCLCPP_INFO(get_logger(), "地面站下发任务模式：%s，开始配置航线。", s.c_str());
  configureForMode();
}

void InventoryMissionNode::configureForMode()
{
  // 按 active_mode_ 建航线并设初始相位。比赛模式由 WAIT_MODE 收到模式后调用；rotate_test 启动即调。
  if (active_mode_ == MissionMode::TRAVERSE) {
    buildTraverseWaypoints();
    current_idx_ = 0;
    phase_ = MissionPhase::TAKEOFF;
  } else if (active_mode_ == MissionMode::ROTATE_TEST) {
    buildRotateTestWaypoints();
    current_idx_ = 0;
    phase_ = MissionPhase::TAKEOFF;
  } else {
    // DIRECTED：先在地面识别抽取码，识别成功后再 buildDirectedWaypoints。
    phase_ = MissionPhase::IDENTIFY;
  }
  RCLCPP_INFO(get_logger(), "航线配置完成，mode=%s，航点数=%zu",
              mode_str_.c_str(), waypoints_.size());
}

// ───────────────────────────── 主循环
void InventoryMissionNode::monitorTimerCallback()
{
  // 先发状态/心跳（含 WAIT_MODE，让地面站一上来就知道飞机已起好、正在待命）。
  publishStatus();

  // WAIT_MODE：上电待命，**不接管控制器、不发目标、不起飞**，纯等地面站 /inventory_mode。
  // （比赛模式飞机重启后停在这里，autostart 上电不会乱起飞——modeCommandCallback 收到才离开。）
  if (phase_ == MissionPhase::WAIT_MODE) {
    return;
  }

  // 真正要起飞那拍才接管控制器 + uart 开门（推迟到此，待命期间不接管飞控）。
  // ⚠ 定向(DIRECTED)模式离开 WAIT_MODE 先进 IDENTIFY 在地面读抽取码、等地面站下发货位，
  //   这段可能好几秒~十几秒，期间**不能**接管——否则位置 PID 已在跑却没有效目标点(用默认0,0,0)
  //   可能让飞机在地面乱动。故排除 IDENTIFY：等识别完进入 TAKEOFF 那拍(有航点了)再接管。
  //   遍历(TRAVERSE)/旋转测试离开待命直接是 TAKEOFF，行为不变、第一拍即接管。
  if (!first_publish_done_ && phase_ != MissionPhase::IDENTIFY) {
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
  // 超时补救中：目标改为后退点（远离板面），其余货位/正常态仍发原航点。
  const InventoryWaypoint tgt = retreat_in_progress_ ? retreatWaypoint(wp) : wp;
  publishTarget(tgt);

  double x_cm, y_cm, yaw_deg;
  if (!getCurrentPose(x_cm, y_cm, yaw_deg)) {
    return;  // 还没拿到位姿，先持续发目标
  }
  double z_cm;
  { std::lock_guard<std::mutex> lk(mutex_); z_cm = current_height_cm_; }

  if (!isReached(tgt, x_cm, y_cm, z_cm, yaw_deg)) {
    return;  // 未到点（含飞向后退点途中），继续逼近
  }

  // 到点。盘点航点要跑盘点子状态；过渡航点直接推进。
  if (wp.scan) {
    if (!scan_active_) {
      // 本货位是否由升/降进入（与上一航点高度不同）→ 决定 qr_vision 是否额外卡纵向：
      // 换行下降/上升才开 strict_vertical（避免下降途中横向恰对正就提前打激光）；
      // 同一行内同高度横移则不开，行为与旧版完全一致。
      // 后退重试时（retreat_in_progress_）若开了 tighten_laser_on_retreat_ 也强制收严。
      const bool vertical_entry =
        current_idx_ > 0 &&
        std::fabs(wp.z_cm - waypoints_[current_idx_ - 1].z_cm) > vertical_entry_tol_cm_;
      const bool strict = vertical_entry ||
                          (retreat_in_progress_ && tighten_laser_on_retreat_);
      scan_qr_aligned_ = false;
      has_qr_id_ = false;
      scan_laser_fired_ = false;
      scan_active_ = true;
      scan_start_time_ = now();
      publishStrictVertical(strict);          // 先发严判开关，再开识别
      publishQrEnable(true);                  // 开识别+激光
    }
    const ScanOutcome oc = runScanAtWaypoint();
    if (oc == ScanOutcome::RECORDED) {
      publishQrEnable(false);
      publishStrictVertical(false);
      scan_active_ = false;
      advance();                              // advance() 内重置 retreat 状态
    } else if (oc == ScanOutcome::TIMED_OUT) {
      if (scan_retreat_cm_ > 0.0 && !scan_retreated_) {
        // 第一次超时 → 后退 scan_retreat_cm 重试（离板远些好解码）。
        scan_retreated_ = true;
        retreat_in_progress_ = true;
        scan_active_ = false;                 // 飞到后退点后会重新 init 扫描
        publishQrEnable(false);               // 飞过去途中先关，避免半路乱打
        publishStrictVertical(false);
        RCLCPP_WARN(get_logger(), "货位 %s 超时，后退 %.0fcm 重试识别。",
                    wp.slot.c_str(), scan_retreat_cm_);
      } else {
        // 已退过仍超时（或关闭了后退）→ 放弃跳过，避免卡死整套。
        RCLCPP_WARN(get_logger(), "货位 %s 重试仍超时，跳过。", wp.slot.c_str());
        publishQrEnable(false);
        publishStrictVertical(false);
        scan_active_ = false;
        advance();
      }
    }
  } else {
    advance();
  }
}

// ───────────────────────────── 盘点子状态
ScanOutcome InventoryMissionNode::runScanAtWaypoint()
{
  const double elapsed = (now() - scan_start_time_).seconds();
  if (elapsed < scan_settle_sec_) {
    return ScanOutcome::WAITING;  // 先稳一下再判定
  }

  bool fired; std::string id;
  { std::lock_guard<std::mutex> lk(mutex_);
    fired = scan_laser_fired_; id = scan_laser_id_; }

  // 完成判据 = qr_vision 回报“激光已打满 0.5s”（/qr_vision/laser_fired），而非松的 aligned。
  // 激光只在横向(及换行时纵向)都进窗才打 → 顺序恒为 对正→打满激光→记录→推进，
  // 不会还没打准就提前飞向下一个货位（修 24fly 下降途中提前识别/打偏的老 bug）。
  if (fired) {
    const int cargo = parseCargoId(id);
    const std::string & slot = waypoints_[current_idx_].slot;
    recordInventory(slot, cargo);

    std_msgs::msg::String res;
    res.data = "编号=" + std::to_string(cargo) + ",货位=" + slot;
    inventory_result_pub_->publish(res);

    std_msgs::msg::Empty led; inventory_led_pub_->publish(led);  // 地面站 LED 亮灭一次

    RCLCPP_INFO(get_logger(), "盘点 %s -> 货物编号 %d", slot.c_str(), cargo);
    return ScanOutcome::RECORDED;
  }

  if (elapsed > scan_timeout_sec_) {
    return ScanOutcome::TIMED_OUT;  // 交由上层决定后退重试还是跳过
  }
  return ScanOutcome::WAITING;
}

// 后退点：沿机头反方向后撤 scan_retreat_cm。相机/激光/机头都朝板面，故板面在 +机头方向，
// 后撤=-(cos yaw, sin yaw)。yaw0(A/C)→x减小；yaw180(B/D)→x增大。y/z 保持不变（只改 standoff）。
InventoryWaypoint InventoryMissionNode::retreatWaypoint(const InventoryWaypoint & wp) const
{
  InventoryWaypoint r = wp;
  const double yaw_rad = wp.yaw_deg * M_PI / 180.0;
  r.x_cm = wp.x_cm - std::cos(yaw_rad) * scan_retreat_cm_;
  r.y_cm = wp.y_cm - std::sin(yaw_rad) * scan_retreat_cm_;
  return r;
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
namespace
{
// 一个货架面的实测几何（坐标系：起飞黑方框中心=(0,0)，蟹行机头顶板面）。
//   x_cm   ：飞机正对该面悬停时的 x（相机/激光到板的 standoff 已含在内，直接是飞机位置）
//   yaw_deg：该面机头朝向（正面 0、背面整机转 180）
//   col_y  ：三列二维码飞机观测点的 y，按 y 递增排列（列0/1/2）
//   z_high / z_low：高行 / 低行二维码对准时的飞机高度
struct FaceGeometry
{
  double x_cm;
  double yaw_deg;
  std::array<double, 3> col_y_cm;
  double z_high_cm;
  double z_low_cm;
  bool   measured;   // false=尚未标定的占位值（货架2 C/D）
};

// ⚠ 场地实测标定（2026-06-05，货架1 两面 A/B；坐标系见上）：
//   A 面：飞机 x=0、机头朝 +x(yaw 0)，三列 y=70/123/173，高行 z=129、低行 z=90。
//   B 面：飞机 x=150、转 180(yaw 180)，三列 y=75/128/178，高行 z=133、低行 z=91。
//   （A/B 两面 y 值略有差异属实测正常，不是公式，故逐面写死。）
//   C/D 面（货架2）尚未标定——下方为占位，盘点货架2 前必须实测填入并把 measured 改 true。
FaceGeometry faceGeometry(char face)
{
  switch (face) {
    case 'A': return { 30.0,   0.0, {{ 70.0, 123.0, 173.0}}, 129.0, 90.0, true};
    case 'B': return {120.0, 180.0, {{ 75.0, 128.0, 178.0}}, 133.0, 91.0, true};
    // 货架2（2026-06-05 实测）：板≈x275，C 面 x230 朝+x、D 面 x320 朝-x。
    case 'C': return {230.0,   0.0, {{ 71.0, 123.0, 174.0}}, 132.0, 90.0, true};
    case 'D': return {320.0, 180.0, {{ 75.0, 127.0, 178.0}}, 132.0, 91.0, true};
    default:  return {  0.0,   0.0, {{ 70.0, 123.0, 173.0}}, 129.0, 90.0, false};
  }
}

// 单货位 y 实测微调（cm）：faceGeometry 的 y 按"列"存、被同列高/低两货位共用，
// 这里覆盖个别货位的 y 偏差（实飞复核后单独调），不影响同列另一个货位。
//   D1：实飞激光偏，单独 -3（同列的 D4 不动）。
double slotYAdjustCm(const std::string & slot)
{
  // 遍历 D 面扫描顺序 D4,D5,D6,D3,D2,D1 → 倒数第二=D2、倒数第一=D1（实飞复核单独微调）。
  if (slot == "D2") return -6.0;   // 倒数第二个：y 减小 6（同列 D5 不动）
  if (slot == "D1") return -6.0;   // 倒数第一个：原 -3 再减小 3 → -6（同列 D4 不动）
  return 0.0;
}
}  // namespace

InventoryWaypoint InventoryMissionNode::slotToScanWaypoint(const std::string & slot) const
{
  // slot = 面字符(A/B/C/D) + 货位号(1..6)，如 "C5"。
  // 货位编号约定：idx 1/2/3 = 高行 列0/1/2，idx 4/5/6 = 低行 列0/1/2（列按 y 递增）。
  // ✅ 货位号↔码对应已确认（2026-06-05，四面统一、无镜像）：上排=1/2/3(高z)、下排=4/5/6(低z)；
  //   同排按飞机观测 y：y 小→1,4 / y 中→2,5 / y 大→3,6。B/D 背面虽转 180°，但按飞机观测 y 算
  //   （非相机画面左右），同规律成立，无需镜像。遍历/定向直飞共用此函数，保证两者落点一致。
  const char face = slot.empty() ? 'A' : slot[0];
  int idx = 1;
  try { idx = std::stoi(slot.substr(1)); } catch (...) { idx = 1; }
  idx = std::max(1, std::min(6, idx));

  const FaceGeometry geo = faceGeometry(face);
  const int row = (idx - 1) / 3;    // 0=高行(1,2,3) 1=低行(4,5,6)
  const int col = (idx - 1) % 3;    // 0,1,2 → y 递增

  InventoryWaypoint wp;
  wp.x_cm    = geo.x_cm;
  wp.y_cm    = geo.col_y_cm[col] + slotYAdjustCm(slot);  // 个别货位 y 单独微调（如 D1 -3）
  wp.z_cm    = (row == 0) ? geo.z_high_cm : geo.z_low_cm;
  wp.yaw_deg = geo.yaw_deg;
  wp.scan    = true;
  wp.slot    = slot;
  wp.tag     = "scan";
  return wp;
}

// ───────────────────────────── 航线构建
void InventoryMissionNode::buildTraverseWaypoints()
{
  waypoints_.clear();
  if (traverse_faces_.empty()) return;

  // ── 蛇形（弓字形）扫描，z 与 y 都走折返以省时（要求1-5 越快越好）──
  //   面间：第 0/2.. 面（A,C）先扫高行、第 1/3.. 面（B,D）先扫低行。
  //         上一面在低行收尾 → 下一面就从低行起步，避免反复上下爬高。
  //   面内：第一行 y 递增(列0→2)，第二行 y 递减(列2→0)。
  //   换面：回到 y 轴 → 沿 x 平移到本面正前方 → 原地旋到本面朝向（背面 yaw 180 在此完成）。
  // 实测验证（2026-06-05，货架1）：
  //   A: (0,70,129)→(0,123,129)→(0,173,129)→(0,173,90)→(0,123,90)→(0,70,90)
  //   B(转180): (150,75,91)→(150,128,91)→(150,178,91)→(150,178,133)→(150,128,133)→(150,75,133)

  // 起飞：在 home 上空升到第一面"先扫行"的盘点高度。
  const FaceGeometry first = faceGeometry(traverse_faces_.front()[0]);
  waypoints_.push_back({home_x_cm_, home_y_cm_, first.z_high_cm, 0.0, false, "", "takeoff"});

  for (std::size_t fi = 0; fi < traverse_faces_.size(); ++fi) {
    const std::string & face = traverse_faces_[fi];
    const FaceGeometry geo = faceGeometry(face[0]);
    if (!geo.measured) {
      RCLCPP_WARN(get_logger(),
                  "面 %s 坐标尚未标定（占位值）！盘点货架2 前必须实测填入 faceGeometry，"
                  "本轮货架1 测试请用 traverse_faces:=A,B", face.c_str());
    }
    const bool high_first = (fi % 2 == 0);
    const double first_z = high_first ? geo.z_high_cm : geo.z_low_cm;

    // 换面过渡（除第一面外）：先回过渡 y(退离板面)、再平移到位、最后原地旋转，分三步避免边走边转。
    if (fi > 0) {
      const FaceGeometry prev = faceGeometry(traverse_faces_[fi - 1][0]);
      waypoints_.push_back({prev.x_cm, transit_y_cm_, first_z, prev.yaw_deg, false, "", "return_axis"});
      waypoints_.push_back({geo.x_cm,  transit_y_cm_, first_z, prev.yaw_deg, false, "", "transit"});
      waypoints_.push_back({geo.x_cm,  transit_y_cm_, first_z, geo.yaw_deg,  false, "", "rotate"});
    }

    // 本面 6 个 scan 航点：先扫行 列0→2（y 递增），后扫行 列2→0（y 递减）。
    // 高行槽号 1..3、低行 4..6（见 slotToScanWaypoint）。
    const int first_base  = high_first ? 1 : 4;
    const int second_base = high_first ? 4 : 1;
    for (int c = 0; c <= 2; ++c)
      waypoints_.push_back(slotToScanWaypoint(face + std::to_string(first_base + c)));
    for (int c = 2; c >= 0; --c)
      waypoints_.push_back(slotToScanWaypoint(face + std::to_string(second_base + c)));
  }

  // 返航 + 降落。
  const FaceGeometry last = faceGeometry(traverse_faces_.back()[0]);
  const bool last_high_first = ((traverse_faces_.size() - 1) % 2 == 0);
  const double end_z = last_high_first ? last.z_low_cm : last.z_high_cm;  // 末面收尾行高度
  if (land_at_circle_) {
    // 要求1 四面全跑：末面(D,x320)收尾后已在货架2 外侧、平移到黑圆不会撞板，故不再退 y 轴，
    // 直接保持末面收尾高度平移到黑圆上空，最后垂直降落。
    waypoints_.push_back({land_x_cm_, land_y_cm_, end_z,           last.yaw_deg, false, "", "return"});
    waypoints_.push_back({land_x_cm_, land_y_cm_, land_height_cm_, last.yaw_deg, false, "", "land"});
  } else {
    // 单货架测试（traverse_faces:=A,B）：在最后一面正前方的 y 轴上降落（终点=最后面 x, y=home）。
    waypoints_.push_back({last.x_cm, home_y_cm_, end_z,           last.yaw_deg, false, "", "return"});
    waypoints_.push_back({last.x_cm, home_y_cm_, land_height_cm_, last.yaw_deg, false, "", "land"});
  }
}

void InventoryMissionNode::buildDirectedWaypoints(const std::string & target_slot)
{
  waypoints_.clear();

  // 地面站已下发货位 → 用与遍历完全相同的几何映射到航点（二维码位置固定，slotToScanWaypoint
  // 给标称坐标，细微偏差由 qr_fine_tune 微调）。
  InventoryWaypoint scan = slotToScanWaypoint(target_slot);
  scan.tag = "scan_directed";

  // ⚠ 不能直线 home→货位→黑圆：那条返航直线会斜穿货架板（差点撞板）。
  //   关键事实：飞机所在的这一列 x 上没有板子（板在 x75/x275，飞机在 x30/120/230/320，离板~45cm）。
  //   故进出都「沿飞机自己这一列 y 方向滑动」越过板子尾端 + 在板子远侧空旷走廊(y=directed_return_y)横移，
  //   全程不穿板，且不回头绕远。z 全程保持盘点高度 scan.z，只在最后落点降下来（不必反复升降）。
  //
  //   去程：home 起飞到 scan.z → 沿 y=home_y 横移到本列 → 原地转向 → 沿本列爬进货位
  //   回程：沿本列爬到远侧走廊 y=directed_return_y → 横移到降落 x(板外) → 到黑圆上空 → 垂直降落

  // 1) 起飞：home 上空升到盘点高度（yaw 0）
  waypoints_.push_back({home_x_cm_, home_y_cm_, scan.z_cm, 0.0, false, "", "takeoff"});

  // 2) 沿 y=home_y 走廊横移到本列 → 原地转到该面朝向 → 沿本列爬进货位盘点（全程 z=scan.z）
  waypoints_.push_back({scan.x_cm, home_y_cm_, scan.z_cm, 0.0,          false, "", "transit"});
  waypoints_.push_back({scan.x_cm, home_y_cm_, scan.z_cm, scan.yaw_deg, false, "", "rotate"});
  waypoints_.push_back(scan);

  // 3) 返航：沿本列爬到远侧走廊 y=directed_return_y(板外空旷) → 横移到降落 x → 到黑圆上空 → 降落。
  //    z 一路保持 scan.z，到黑圆正上方才垂直降落。
  waypoints_.push_back({scan.x_cm,  directed_return_y_cm_, scan.z_cm,       scan.yaw_deg, false, "", "return_axis"});
  waypoints_.push_back({land_x_cm_, directed_return_y_cm_, scan.z_cm,       0.0,          false, "", "transit"});
  waypoints_.push_back({land_x_cm_, land_y_cm_,            scan.z_cm,       0.0,          false, "", "return"});
  waypoints_.push_back({land_x_cm_, land_y_cm_,            land_height_cm_, 0.0,          false, "", "land"});
}

void InventoryMissionNode::buildRotateTestWaypoints()
{
  // 纯飞控验证：全部为过渡航点（scan=false，不开识别/激光）。
  //   1) 起飞到 test_height（home 上空，yaw 0）
  //   2) 沿 map +x 前进 test_forward（yaw 0）
  //   3) 原地分步旋转 0 → test_yaw（位置不动，只转 yaw），转到后保持该朝向
  //   4) 保持 test_yaw 朝向平移返航
  //   5) 在 home 垂直降落（保持 test_yaw）
  //
  // 旋转分步（test_yaw_step_deg）：限制单步偏航误差，配合 PID max_angular_velocity 让转速更柔和、
  // 每步可停下观察。想更慢：给位置PID launch 传更小的 max_angular_velocity（默认 30°/s）。
  //
  // ⚠ 这一版「转到 test_yaw 后带偏航平移返航」会**真正触发**带偏航平移：PID 发的 /target_velocity
  //   是 map 系，uart_to_stm32(帧 0x31) 未旋到机体系，若 STM32 固件按机体系解释则返航会反向。
  //   这正是盘点扫背面（yaw 180）要用的同款动作，所以本测试顺带把它验证了——首飞务必小油门、
  //   人随时接管，重点看「转到 180 后往 home 飞时方向对不对」。
  waypoints_.clear();
  const double fwd_x = home_x_cm_ + test_forward_cm_;

  // 1) 起飞 + 2) 前进（yaw 0）
  waypoints_.push_back({home_x_cm_, home_y_cm_, test_height_cm_, 0.0, false, "", "takeoff"});
  waypoints_.push_back({fwd_x,      home_y_cm_, test_height_cm_, 0.0, false, "", "forward"});

  // 3) 原地分步旋转：0 → test_yaw（x,y,z 不变，只改 yaw）
  const double sign = (test_yaw_deg_ >= 0.0) ? 1.0 : -1.0;
  const double goal = std::fabs(test_yaw_deg_);
  const double step = std::max(1.0, std::fabs(test_yaw_step_deg_));
  const int    n    = (goal < 1e-6) ? 0 : static_cast<int>(std::ceil(goal / step));
  for (int i = 1; i <= n; ++i) {
    const double y = sign * std::min(goal, i * step);
    waypoints_.push_back({fwd_x, home_y_cm_, test_height_cm_, y, false, "", "rotate"});
  }

  // 4) 保持 test_yaw 平移返航 + 5) 垂直降落（保持 test_yaw）
  waypoints_.push_back({home_x_cm_, home_y_cm_, test_height_cm_, test_yaw_deg_, false, "", "return"});
  waypoints_.push_back({home_x_cm_, home_y_cm_, land_height_cm_, test_yaw_deg_, false, "", "land"});
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

std::string InventoryMissionNode::buildStatusText() const
{
  // 模式中文标签
  std::string m;
  switch (active_mode_) {
    case MissionMode::TRAVERSE:    m = "遍历"; break;
    case MissionMode::DIRECTED:    m = "定向"; break;
    case MissionMode::ROTATE_TEST: m = "旋转测试"; break;
  }

  if (phase_ == MissionPhase::WAIT_MODE) {
    return "待命中：等地面站下发任务模式（普通/进阶）";
  }
  if (phase_ == MissionPhase::IDENTIFY) {
    if (!target_identified_) return "[定向] 识别抽取码中…（请把抽取码举到机头相机前）";
    if (!has_target_slot_)
      return "[定向] 已识别抽取码 编号" + std::to_string(target_cargo_id_) + "，等地面站下发货位";
    return "[定向] 已收到货位 " + target_slot_ + "，准备起飞";
  }
  if (phase_ == MissionPhase::DONE) {
    return "[" + m + "] 任务完成";
  }
  // 飞行中：按当前航点 tag/slot 给出可读阶段
  if (current_idx_ < waypoints_.size()) {
    const InventoryWaypoint & wp = waypoints_[current_idx_];
    const std::string t(wp.tag ? wp.tag : "");
    if (wp.scan)
      return "[" + m + "] 盘点中 货位" + wp.slot +
             (retreat_in_progress_ ? "（后退重试）" : "");
    if (t == "takeoff")     return "[" + m + "] 起飞中";
    if (t == "return")      return "[" + m + "] 返航中";
    if (t == "land")        return "[" + m + "] 降落中";
    if (t == "rotate")      return "[" + m + "] 转向对面中";
    if (t == "transit" || t == "return_axis") return "[" + m + "] 换面飞行中";
    return "[" + m + "] 飞行中";
  }
  return "[" + m + "] 飞行中";
}

void InventoryMissionNode::publishStatus()
{
  const std::string text = buildStatusText();
  const rclcpp::Time t = now();
  // 内容变了立刻发；否则每 ~0.5s 发一次充当心跳（地面站断流 >N 秒即判离线）。
  if (status_inited_ && text == last_status_text_ &&
      (t - last_status_pub_time_).seconds() < 0.5) {
    return;
  }
  std_msgs::msg::String msg; msg.data = text;
  inventory_status_pub_->publish(msg);
  last_status_text_ = text;
  last_status_pub_time_ = t;
  status_inited_ = true;
}

void InventoryMissionNode::publishQrEnable(bool on)
{
  std_msgs::msg::Bool msg; msg.data = on;
  qr_enable_pub_->publish(msg);
}

void InventoryMissionNode::publishStrictVertical(bool on)
{
  std_msgs::msg::Bool msg; msg.data = on;
  qr_strict_vertical_pub_->publish(msg);
}

void InventoryMissionNode::advance()
{
  // 后退补救是“逐货位”的：推进到下一个航点即复位，下一个货位从正常 standoff 重新开始。
  scan_retreated_ = false;
  retreat_in_progress_ = false;
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
