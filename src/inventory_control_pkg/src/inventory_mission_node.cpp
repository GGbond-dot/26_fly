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
  } else if (active_mode_ == MissionMode::ROTATE_TEST) {
    buildRotateTestWaypoints();
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
    case 'A': return {  0.0,   0.0, {{ 70.0, 123.0, 173.0}}, 129.0, 90.0, true};
    case 'B': return {150.0, 180.0, {{ 75.0, 128.0, 178.0}}, 133.0, 91.0, true};
    // TODO(标定)：货架2 C/D 实测后替换占位，measured 改 true。
    case 'C': return {300.0,   0.0, {{ 70.0, 123.0, 173.0}}, 129.0, 90.0, false};
    case 'D': return {450.0, 180.0, {{ 75.0, 128.0, 178.0}}, 133.0, 91.0, false};
    default:  return {  0.0,   0.0, {{ 70.0, 123.0, 173.0}}, 129.0, 90.0, false};
  }
}
}  // namespace

InventoryWaypoint InventoryMissionNode::slotToScanWaypoint(const std::string & slot) const
{
  // slot = 面字符(A/B/C/D) + 货位号(1..6)，如 "C5"。
  // 货位编号约定：idx 1/2/3 = 高行 列0/1/2，idx 4/5/6 = 低行 列0/1/2（列按 y 递增）。
  // ⚠ 货位号↔真实二维码的对应仍待标定（B/D 背面尤其要核），但**飞机观测坐标**已是实测值，
  //   遍历/定向直飞共用此函数，保证两者落点一致。
  const char face = slot.empty() ? 'A' : slot[0];
  int idx = 1;
  try { idx = std::stoi(slot.substr(1)); } catch (...) { idx = 1; }
  idx = std::max(1, std::min(6, idx));

  const FaceGeometry geo = faceGeometry(face);
  const int row = (idx - 1) / 3;    // 0=高行(1,2,3) 1=低行(4,5,6)
  const int col = (idx - 1) % 3;    // 0,1,2 → y 递增

  InventoryWaypoint wp;
  wp.x_cm    = geo.x_cm;
  wp.y_cm    = geo.col_y_cm[col];
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

    // 换面过渡（除第一面外）：先回 y 轴、再平移到位、最后原地旋转，分三步避免边走边转。
    if (fi > 0) {
      const FaceGeometry prev = faceGeometry(traverse_faces_[fi - 1][0]);
      waypoints_.push_back({prev.x_cm, home_y_cm_, first_z, prev.yaw_deg, false, "", "return_axis"});
      waypoints_.push_back({geo.x_cm,  home_y_cm_, first_z, prev.yaw_deg, false, "", "transit"});
      waypoints_.push_back({geo.x_cm,  home_y_cm_, first_z, geo.yaw_deg,  false, "", "rotate"});
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

  // 返航 + 降落。本轮（货架1 两面测试）在最后一面正前方的 y 轴上降落（终点=最后面 x, y=home）。
  // TODO(正式赛)：四面全跑时降落点应为黑圆 land_x_cm_/land_y_cm_，标定后切回。
  const FaceGeometry last = faceGeometry(traverse_faces_.back()[0]);
  const bool last_high_first = ((traverse_faces_.size() - 1) % 2 == 0);
  const double end_z = last_high_first ? last.z_low_cm : last.z_high_cm;  // 末面收尾行高度
  waypoints_.push_back({last.x_cm, home_y_cm_, end_z,           last.yaw_deg, false, "", "return"});
  waypoints_.push_back({last.x_cm, home_y_cm_, land_height_cm_, last.yaw_deg, false, "", "land"});
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
