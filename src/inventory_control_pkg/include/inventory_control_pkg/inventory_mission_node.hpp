#pragma once

#include <array>
#include <cstddef>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace inventory_control_pkg
{

// 一个货位（A1..D6）对应的盘点航点 / 一个过渡航点。
//   scan=true  → 到点悬停，开识别，等对准+读到二维码，记录货物编号→货位、闪 LED、发结果
//   scan=false → 普通过渡航点（起飞/换面/换货架/返航）
//
// 单相机 + yaw 旋转方案：扫 A 面时 yaw 朝向 A 面，扫 B 面时整机 yaw 旋转 180° 朝 B 面。
// 每个货位的 (x,y,z,yaw) 由 buildTraverseWaypoints() 按场地几何算出（见 cpp 内 TODO）。
struct InventoryWaypoint
{
  double x_cm;
  double y_cm;
  double z_cm;
  double yaw_deg;
  bool   scan;
  std::string slot;   // "A1".."D6"，过渡航点为空
  const char * tag;
};

enum class MissionMode
{
  TRAVERSE,    // 要求1：遍历盘点 2 货架 4 面 24 码
  DIRECTED,    // 要求2：先识别抽取码 → 只飞去那一个货位盘点
  ROTATE_TEST  // 调试：起飞→前进→原地 yaw 180°→返航→降落，验证旋转+平移飞控（不开识别/激光）
};

enum class MissionPhase
{
  WAIT_MODE,   // 上电后原地待命（不起飞、不接管控制器），等地面站发 /inventory_mode 告知 普通/进阶
  IDENTIFY,    // 仅 DIRECTED：上电后在地面识别"抽取的那张二维码"，报送编号给地面站
  TAKEOFF,     // 起飞点垂直起飞，升至 150cm
  TRAVERSE,    // 按航点队列逐货位盘点（含 yaw 换面）
  RETURN,      // 飞回降落点上空
  LAND,        // 垂直降落到圆形降落点
  DONE
};

// 单货位盘点子状态的一次判定结果。
enum class ScanOutcome
{
  WAITING,   // 还在稳定/等对准打激光
  RECORDED,  // 已对准+激光打满+记录，可推进
  TIMED_OUT  // 超时仍没读到/打到，交由上层决定后退重试还是跳过
};

// D 题 立体货架盘点任务状态机。
//
// 接口约定（复用本仓 PID/uart 链路，与 spray_control_pkg 同款）：
//   发布 /target_position   Float32MultiArray = [x_cm, y_cm, z_cm, yaw_deg]
//   发布 /active_controller UInt8 (2=位置控制器接管, 0=停)
//   发布 /route_choice      UInt8 (1 → uart_to_stm32 开门转发 /target_velocity 到飞控)
//   发布 /qr_vision/enable  Bool  (到货位才 true，开识别+激光；平时 false)
//   发布 /qr_vision/strict_vertical Bool (本货位由升/降进入时 true，激光额外卡纵向；同高度横移 false)
//   发布 /inventory_result  String("编号=7,货位=B3") → 地面站 LCD 实时显示
//   发布 /inventory_led     Empty → 地面站每盘到一个货物亮灭一次 LED
//   发布 /inventory_target  String → DIRECTED：先报送抽取码编号("7")，地面站下发货位后
//                                    再报送"目标编号=7,货位=C5"（供地面站画航线图，要求2-2）
//   发布 /mission_complete  Empty
//   订阅 /height            Int16 (cm，uart_to_stm32 上报)
//   订阅 /qr_vision/id      String (识别到的货物编号 "1".."24")
//   订阅 /qr_vision/aligned Bool   (二维码已对准画面中心)
//   订阅 /qr_vision/laser_fired String (激光打满0.5s后回报刚打的码，本货位据此判“已真打”才推进)
//   订阅 /inventory_target_slot String → DIRECTED：地面站查"编号→货位"表后下发货位("C5")
//   位姿通过 tf2: map → laser_link
class InventoryMissionNode : public rclcpp::Node
{
public:
  explicit InventoryMissionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ── 订阅回调 ──
  void heightCallback(const std_msgs::msg::Int16::SharedPtr msg);
  void qrIdCallback(const std_msgs::msg::String::SharedPtr msg);
  void qrAlignedCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void laserFiredCallback(const std_msgs::msg::String::SharedPtr msg);

  void monitorTimerCallback();

  bool getCurrentPose(double & x_cm, double & y_cm, double & yaw_deg);
  bool isReached(const InventoryWaypoint & wp, double x_cm, double y_cm,
                 double z_cm, double yaw_deg) const;

  void publishTarget(const InventoryWaypoint & wp);
  void publishQrEnable(bool on);
  void publishStrictVertical(bool on);
  void advance();

  // 往地面站发"状态/心跳"（/inventory_status, String）。地面站据此显示飞机是否已起好、
  // 待命/识别抽取码/盘点到哪一步——操作员只能看地面站，靠这条掌握远端飞机状态。
  // 内容随阶段变化，且至少每 ~0.5s 发一次充当心跳（断流即判离线）。
  void publishStatus();
  std::string buildStatusText() const;

  // 盘点子状态：到达 scan 航点后调用，悬停等"对准 + 读到码"。
  // 返回 RECORDED=完成(已记录+发结果+闪LED)；TIMED_OUT=超时；WAITING=继续等。
  ScanOutcome runScanAtWaypoint();

  // 超时补救：把盘点航点沿"机头反方向"后撤 scan_retreat_cm（远离板面）得到重试点。
  // 相机/激光/机头都朝板面，故后撤方向 = -(cos yaw, sin yaw)：A/C(yaw0)→x减小、B/D(yaw180)→x增大。
  InventoryWaypoint retreatWaypoint(const InventoryWaypoint & wp) const;

  // DIRECTED 起飞前：地面识别抽取的那张码，拿到编号后报送地面站并构建单货位航线。
  // 返回 true 表示已识别成功。
  bool runIdentifyTarget();

  // 构建遍历航线：起飞→A1..A6→yaw转→B1..B6→飞货架2→C1..C6→yaw转→D1..D6→返航。
  void buildTraverseWaypoints();
  // 构建定向航线：起飞→直飞目标货位→盘点→返航。target_slot 由识别结果映射得到。
  void buildDirectedWaypoints(const std::string & target_slot);
  // 构建旋转测试航线（调试用，不开识别/激光）：
  //   起飞到 test_height → 前进 test_forward → 原地 yaw 转 test_yaw → 返航 → 降落。
  void buildRotateTestWaypoints();

  // 货位（"A1".."D6"）→ 盘点航点。遍历与定向共用此函数，保证"定向直飞的点"
  // 与"遍历记录该货位时的点"严格一致。几何来自题目 图1/图2（仍需场地标定 y_center 等）。
  InventoryWaypoint slotToScanWaypoint(const std::string & slot) const;

  // 把"货物编号"映射到货位（A1..D6）。遍历盘点时由识别顺序直接得到货位；
  // 定向盘点的"编号→货位"由地面站权威下发（见 targetSlotCallback）。
  void recordInventory(const std::string & slot, int cargo_id);

  // 要求2：地面站收到飞机报送的抽取码编号后，查它自己的表得出货位，下发给飞机。
  void targetSlotCallback(const std_msgs::msg::String::SharedPtr msg);

  // 地面站下发任务模式（"traverse"/"directed"）。飞机重启后在 WAIT_MODE 等这条，收到才配置起飞。
  void modeCommandCallback(const std_msgs::msg::String::SharedPtr msg);
  // 按 active_mode_ 配置初始航线/相位（WAIT_MODE 收到模式、或 rotate_test 启动时调用）。
  void configureForMode();

  static double meterToCm(double v) { return v * 100.0; }
  double normalizeAngleDeg(double angle_deg) const;
  int parseCargoId(const std::string & text) const;

  // ── 参数 ──
  std::string map_frame_;
  std::string base_frame_;            // laser_link（与 PID/uart 一致）

  std::string mode_str_;              // "traverse" / "directed"
  MissionMode mode_;

  // 遍历哪些面（逗号分隔，如 "A,B,C,D" 全跑；"A,B" 只跑货架1，第二货架未到时用）。
  std::string traverse_faces_str_;
  std::vector<std::string> traverse_faces_;

  double pos_tol_cm_;
  double yaw_tol_deg_;
  double height_tol_cm_;

  double flight_height_cm_;           // 巡航/盘点高度，默认 150
  double land_height_cm_;
  double home_x_cm_;                  // 起飞点"十"字坐标
  double home_y_cm_;
  double land_x_cm_;                  // 圆形降落点坐标（与起飞点不同）
  double land_y_cm_;
  bool   land_at_circle_;             // true=遍历跑完飞黑圆(land_x/y)降落；false=落末面正前方y轴(单货架测试)
  double transit_y_cm_;               // 换面过渡航点的 y（往后退离板面，默认 -20，不贴 y=0）
  // 定向返航横移走廊的 y（板子远侧、空旷）：沿本列爬到此 y 再横移到降落 x，最后降到 land_y。
  // 默认 300（板子 y≤~180，留足余量），改了不重编。
  double directed_return_y_cm_;

  // 盘点子状态参数
  double scan_settle_sec_;            // 到位后先稳定再开识别
  double scan_timeout_sec_;           // 单货位最长等待，超时后退重试或跳过避免卡死
  double scan_retreat_cm_;            // 超时后沿机头反方向后撤多少 cm 再试一次（0=不后退直接跳过）
  bool   tighten_laser_on_retreat_;   // 后退重试时强制开 strict_vertical 收严激光（standoff 变大窗口变松）
  double vertical_entry_tol_cm_;      // 与上一航点高度差超此值=换行(升/降)进入→开纵向严判
  double led_blink_sec_;              // LED 亮灭时长（题目要求约 1s）

  // 货架几何（cm，来自题目 图1/图2，buildTraverseWaypoints 用）
  double shelf1_x_cm_;                // 货架1（A/B面）红杆中线 x
  double shelf2_x_cm_;                // 货架2（C/D面）红杆中线 x
  double slot_col_spacing_cm_;        // 同面相邻列间距 50
  double slot_row_spacing_cm_;        // 同面上下行间距 80（上行105、下行25 之类，见 cpp）
  double scan_standoff_cm_;           // 相机离板面的水平后撤距离（视场决定）

  // 旋转测试航线参数（mode:=rotate_test，纯飞控验证，可在 launch 改无需重编）
  double test_height_cm_;             // 测试悬停高度（默认 100）
  double test_forward_cm_;            // 沿 map +x 前进距离（默认 200）
  double test_yaw_deg_;              // 原地旋转目标偏航（默认 180）
  double test_yaw_step_deg_;          // 偏航分步步长（默认 180=连续转）；设更小值则分段转，每步停一下

  // ── 航点 / 阶段 ──
  std::vector<InventoryWaypoint> waypoints_;
  MissionMode active_mode_;
  MissionPhase phase_;
  std::size_t  current_idx_;

  // 盘点子状态
  bool          scan_active_;
  rclcpp::Time  scan_start_time_;
  bool          scan_qr_aligned_;
  std::string   latest_qr_id_;
  bool          has_qr_id_;
  bool          scan_laser_fired_;     // 本货位 qr_vision 已回报激光打满（/qr_vision/laser_fired）
  std::string   scan_laser_id_;        // 回报里刚打的那张码（作为本货位编号记录）
  bool          scan_retreated_;       // 本货位已做过一次"后退重试"（避免无限后退；每货位只退一次）
  bool          retreat_in_progress_;  // 正在飞向/停在后退点重试（publishTarget 改发后退点）

  // DIRECTED 目标
  bool          target_identified_;     // 已读到抽取码编号并报送地面站
  int           target_cargo_id_;
  bool          has_target_slot_;        // 地面站已下发货位
  std::string   target_slot_;            // 地面站下发的货位（"A1".."D6"）

  // 盘点结果：slot → cargo_id
  std::map<std::string, int> inventory_;

  // ── ROS ──
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr target_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr             active_controller_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr             route_choice_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr             qr_enable_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr             qr_strict_vertical_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr           inventory_result_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr            inventory_led_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr           inventory_target_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr            mission_complete_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr           inventory_status_pub_;  // 状态/心跳→地面站

  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr   height_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr  qr_id_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr    qr_aligned_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr  qr_laser_fired_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr  target_slot_sub_;  // 地面站下发货位
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr  mode_cmd_sub_;     // 地面站下发任务模式

  rclcpp::TimerBase::SharedPtr  monitor_timer_;

  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  mutable std::mutex mutex_;
  bool   has_height_;
  double current_height_cm_;
  bool   mission_complete_sent_;
  bool   first_publish_done_;

  // 状态心跳节流：内容变了立刻发，否则每 ~0.5s 发一次。
  std::string  last_status_text_;
  rclcpp::Time last_status_pub_time_;
  bool         status_inited_;
};

}  // namespace inventory_control_pkg
