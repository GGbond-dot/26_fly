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
  TRAVERSE,   // 要求1：遍历盘点 2 货架 4 面 24 码
  DIRECTED    // 要求2：先识别抽取码 → 只飞去那一个货位盘点
};

enum class MissionPhase
{
  IDENTIFY,    // 仅 DIRECTED：上电后在地面识别"抽取的那张二维码"，报送编号给地面站
  TAKEOFF,     // 起飞点垂直起飞，升至 150cm
  TRAVERSE,    // 按航点队列逐货位盘点（含 yaw 换面）
  RETURN,      // 飞回降落点上空
  LAND,        // 垂直降落到圆形降落点
  DONE
};

// D 题 立体货架盘点任务状态机。
//
// 接口约定（复用本仓 PID/uart 链路，与 spray_control_pkg 同款）：
//   发布 /target_position   Float32MultiArray = [x_cm, y_cm, z_cm, yaw_deg]
//   发布 /active_controller UInt8 (2=位置控制器接管, 0=停)
//   发布 /route_choice      UInt8 (1 → uart_to_stm32 开门转发 /target_velocity 到飞控)
//   发布 /qr_vision/enable  Bool  (到货位才 true，开识别+激光；平时 false)
//   发布 /inventory_result  String("编号=7,货位=B3") → 地面站 LCD 实时显示
//   发布 /inventory_led     Empty → 地面站每盘到一个货物亮灭一次 LED
//   发布 /inventory_target  String → DIRECTED 模式报送"抽取码"的编号给地面站
//   发布 /mission_complete  Empty
//   订阅 /height            Int16 (cm，uart_to_stm32 上报)
//   订阅 /qr_vision/id      String (识别到的货物编号 "1".."24")
//   订阅 /qr_vision/aligned Bool   (二维码已对准画面中心)
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

  void monitorTimerCallback();

  bool getCurrentPose(double & x_cm, double & y_cm, double & yaw_deg);
  bool isReached(const InventoryWaypoint & wp, double x_cm, double y_cm,
                 double z_cm, double yaw_deg) const;

  void publishTarget(const InventoryWaypoint & wp);
  void publishQrEnable(bool on);
  void advance();

  // 盘点子状态：到达 scan 航点后调用，悬停等"对准 + 读到码"。
  // 返回 true 表示本货位盘点完成（已记录 + 已发结果 + 已闪 LED），可推进。
  bool runScanAtWaypoint();

  // DIRECTED 起飞前：地面识别抽取的那张码，拿到编号后报送地面站并构建单货位航线。
  // 返回 true 表示已识别成功。
  bool runIdentifyTarget();

  // 构建遍历航线：起飞→A1..A6→yaw转→B1..B6→飞货架2→C1..C6→yaw转→D1..D6→返航。
  void buildTraverseWaypoints();
  // 构建定向航线：起飞→直飞目标货位→盘点→返航。target_slot 由识别结果映射得到。
  void buildDirectedWaypoints(const std::string & target_slot);

  // 把"货物编号"映射到货位（A1..D6）。遍历盘点时由识别顺序直接得到货位；
  // 定向盘点时若只知编号、不知货位，需要先飞一轮或靠先验表（见 cpp TODO）。
  void recordInventory(const std::string & slot, int cargo_id);

  static double meterToCm(double v) { return v * 100.0; }
  double normalizeAngleDeg(double angle_deg) const;
  int parseCargoId(const std::string & text) const;

  // ── 参数 ──
  std::string map_frame_;
  std::string base_frame_;            // laser_link（与 PID/uart 一致）

  std::string mode_str_;              // "traverse" / "directed"
  MissionMode mode_;

  double pos_tol_cm_;
  double yaw_tol_deg_;
  double height_tol_cm_;

  double flight_height_cm_;           // 巡航/盘点高度，默认 150
  double land_height_cm_;
  double home_x_cm_;                  // 起飞点"十"字坐标
  double home_y_cm_;
  double land_x_cm_;                  // 圆形降落点坐标（与起飞点不同）
  double land_y_cm_;

  // 盘点子状态参数
  double scan_settle_sec_;            // 到位后先稳定再开识别
  double scan_timeout_sec_;           // 单货位最长等待，超时跳过避免卡死
  double led_blink_sec_;              // LED 亮灭时长（题目要求约 1s）

  // 货架几何（cm，来自题目 图1/图2，buildTraverseWaypoints 用）
  double shelf1_x_cm_;                // 货架1（A/B面）红杆中线 x
  double shelf2_x_cm_;                // 货架2（C/D面）红杆中线 x
  double slot_col_spacing_cm_;        // 同面相邻列间距 50
  double slot_row_spacing_cm_;        // 同面上下行间距 80（上行105、下行25 之类，见 cpp）
  double scan_standoff_cm_;           // 相机离板面的水平后撤距离（视场决定）

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

  // DIRECTED 目标
  bool          target_identified_;
  int           target_cargo_id_;
  std::string   target_slot_;

  // 盘点结果：slot → cargo_id
  std::map<std::string, int> inventory_;

  // ── ROS ──
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr target_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr             active_controller_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr             route_choice_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr             qr_enable_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr           inventory_result_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr            inventory_led_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr           inventory_target_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr            mission_complete_pub_;

  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr   height_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr  qr_id_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr    qr_aligned_sub_;

  rclcpp::TimerBase::SharedPtr  monitor_timer_;

  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  mutable std::mutex mutex_;
  bool   has_height_;
  double current_height_cm_;
  bool   mission_complete_sent_;
  bool   first_publish_done_;
};

}  // namespace inventory_control_pkg
