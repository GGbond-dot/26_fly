#pragma once

#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace fire_control_pkg
{

// 一个巡逻航点。坐标一律用**题目场地系 dm**（左下角为原点，右上角 (48,40)），
// 只在 publishTarget() / getCurrentPose() 两处与 map 系 cm 互转（见 fieldToMap / mapToField）。
// 这样航线定义可以直接照题目图 1 抄，发给消防车的坐标也不用再换算。
struct FireWaypoint
{
  double x_dm;
  double y_dm;
  double z_dm;        // 离地高度（dm），与 /height 直接可比，不参与坐标系变换
  double yaw_deg;
  int    strip_id;    // 属于第几条巡逻带（0 起；过渡航点为 -1）
  const char * tag;
};

enum class MissionPhase
{
  WAIT_START,  // 上电待命：不起飞、不接管控制器，等消防车按键 → /fire_start
  TAKEOFF,     // 起降点（左下黑色区）垂直起飞，升至巡航高度 18dm
  PATROL,      // 蛇形全覆盖巡逻；期间 1Hz 向消防车播位置
  APPROACH,    // 发挥1：巡航高度粗对中——闭环把火源修到画面中心，开机上 LED 报警
  DESCEND,     // 发挥2：降到 10dm
  SERVO_FINE,  // 发挥2：10dm 精对中（目标更大，像素分辨率更高）
  HOVER,       // 发挥2：悬停 3s
  DROP,        // 发挥2：抛洒灭火包（/drop_package → 飞控舵机帧 0x11）
  RESUME,      // 拉回巡航高度，回到被打断的航点继续巡逻
  RETURN,      // 飞回起降点上空
  LAND,        // 垂直降落到起飞区内
  DONE
};

// 空地协同智能消防系统（2023 电赛 G 题）无人机侧任务状态机。
//
// 接口约定（复用本仓 PID/uart 链路，与 spray_control_pkg / inventory_control_pkg 同款）：
//   发布 /target_position       Float32MultiArray = [x_cm, y_cm, z_cm, yaw_deg]（**map 系**）
//   发布 /active_controller     UInt8 (2=位置控制器接管, 0=停)
//   发布 /route_choice          UInt8 (1 → uart_to_stm32 开门转发 /target_velocity 到飞控)
//   发布 /magnet/cmd            Int32 (1=机腹激光笔亮, 2=灭) → magnet_control_pkg
//                               香橙派 40pin(PB0) + WiringOP `gpio write`，低电平点亮。
//                               本题激光只用于指示航迹，起飞即常亮、降落才灭，无闪烁时序。
//   发布 /buzzer_led_control    UInt8 (1=机上 LED 报警亮, 0=灭)，帧 0x22
//   发布 /drop_package          UInt8 (1=抛洒灭火包→舵机1400开, 0=复位→700关)，飞控舵机帧 0x11。
//                               开合时序在本节点做（drop_pulse_sec_），固件只认单字节开/关。
//   发布 /drone_pose            Float32MultiArray [x_dm, y_dm] @1Hz → 消防车实时显示（基本要求3）
//   发布 /patrol_distance       Float32MultiArray [dist_dm] @1Hz → 累计巡逻里程（基本要求4，
//                               车端自己也能积分，机端这份用位姿直接算更准，两边对照）
//   发布 /fire_report           Float32MultiArray [x_dm, y_dm] → 火源坐标发消防车（发挥2）
//   发布 /fire_status           String → 状态/心跳，供地面观察飞机走到哪一步
//   发布 /mission_complete      Empty
//   订阅 /fire_start            Empty  ← 消防车按键启动（经 UDP 桥转成本地话题）
//   订阅 /height                Int16 (cm，uart_to_stm32 上报)
//   订阅 /fire/detected         Bool   ← fire_vision_pkg：本帧看到红色火花图案
//   订阅 /fire/position_map     Float32MultiArray [x_cm, y_cm]（**map 系**）
//                               ← fire_vision_pkg 用像素+高度反投影得到的火源地面坐标
//   位姿通过 tf2: map → laser_link
//
// 目标判定：场上三个火源图案中**只有目标是红色**，另两个是其它颜色，
// 故本节点不做筛选——收到 fire_vision 的确认即认定为唯一目标，处理完置 fire_handled_ 不再响应。
class FireMissionNode : public rclcpp::Node
{
public:
  explicit FireMissionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ── 订阅回调 ──
  void startCallback(const std_msgs::msg::Empty::SharedPtr msg);
  void heightCallback(const std_msgs::msg::Int16::SharedPtr msg);
  void fireDetectedCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void firePositionCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void servoErrorCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

  // 视觉伺服一步：把机体系偏移换算成 map 目标点发出去。返回 true 表示已进死区。
  // 相机与抛投口视为同 z 轴，故"火源在画面中心"即"火源在抛投口正下方"。
  bool servoStep(double z_dm, const char * tag);

  void monitorTimerCallback();
  void telemetryTimerCallback();   // 1Hz：位置 + 累计里程 → 消防车

  // 场地系 dm ↔ map 系 cm。假定起飞时机头朝场地 +x，即两系只差一个平移。
  // 若现场摆放导致机头不朝 +x，改 field_yaw_offset_deg_ 参数即可（会带旋转）。
  void fieldToMap(double x_dm, double y_dm, double & x_cm, double & y_cm) const;
  void mapToField(double x_cm, double y_cm, double & x_dm, double & y_dm) const;

  bool getCurrentPose(double & x_dm, double & y_dm, double & yaw_deg);
  bool isReached(const FireWaypoint & wp, double x_dm, double y_dm,
                 double z_dm, double yaw_deg) const;

  void publishTarget(const FireWaypoint & wp);
  void publishLaser(bool on);
  void publishAlarmLed(bool on);
  void publishDrop(bool on);
  // 火源坐标 → 消防车（经 fire_link_pkg 转 UDP）。发一次即可：
  // 丢包冗余在 fire_link_node 里做（同一 seq 连发 report_repeat 次），这里再循环
  // 会变成多个不同 seq 的独立事件，与《G题_机车通信接口约定.md》§二 不符。
  void publishFireReport(const char * reason);
  void advance();

  // 状态/心跳（/fire_status）。内容变了立刻发，否则每 ~0.5s 发一次充当心跳。
  void publishStatus();
  std::string buildStatusText() const;

  // 蛇形全覆盖航线：strip_y_dm 每条带一行，x 在 [patrol_x_min_dm, patrol_x_max_dm]
  // 之间往返。激光笔垂直向下，扫过即覆盖，故一条带只需两个端点航点。
  void buildPatrolWaypoints();

  // 发挥流程：把当前航点索引存下来，插入"火源上方"这一串临时目标；
  // RESUME 结束后回到 patrol_resume_idx_ 继续。
  void enterFireHandling();
  FireWaypoint fireWaypoint(double z_dm, const char * tag) const;

  // 巡逻里程累计：每次位姿更新，与上一采样点的距离累加（超过 dist_sample_min_dm 才计，滤抖动）。
  void accumulateDistance(double x_dm, double y_dm);

  static double cmToDm(double v) { return v / 10.0; }
  static double dmToCm(double v) { return v * 10.0; }
  double normalizeAngleDeg(double angle_deg) const;

  // ── 参数 ──
  std::string map_frame_;
  std::string base_frame_;            // laser_link（与 PID/uart 一致）

  // 场地 ↔ map 对齐：起飞点在**场地系**下的坐标（左下黑色起降区中心，实测 7×7dm → (3.5,3.5)）
  double home_field_x_dm_;
  double home_field_y_dm_;
  double field_yaw_offset_deg_;       // 场地 +x 相对 map +x 的夹角，正常摆放为 0

  double pos_tol_dm_;
  double yaw_tol_deg_;
  double height_tol_dm_;

  // 飞行高度（dm，题目单位）
  double patrol_height_dm_;           // 巡逻高度，题目要求 18dm 左右
  double drop_height_dm_;             // 抛包高度，题目要求 10dm 左右
  double land_height_dm_;

  // 巡逻航线
  std::vector<double> strip_y_dm_;    // 各条带中心线 y，默认 {4,12,20,28,36}
  double patrol_x_min_dm_;
  double patrol_x_max_dm_;
  double patrol_yaw_deg_;             // 全程固定偏航，默认 0（不转向，省时且避开 ±180 跳变）

  // 发挥部分
  bool   enable_fire_task_;           // 基本要求单独试飞时关掉，只跑 起飞→巡逻→返航→降落
  double fire_confirm_sec_;           // 火源位置需稳定这么久才认（配合 vision 侧投票）
  double fire_position_jitter_dm_;    // 稳定判据：期间位置抖动不超过此值
  double hover_before_drop_sec_;      // 抛包前悬停，题目要求 3s
  double drop_pulse_sec_;             // /drop_package=1 保持时长，之后回 0

  // ── 视觉伺服（闭环对中）──
  // 开环反投影的误差直接变成抛包误差，且 fx/fy、/height、cam_offset 全都要标定。
  // 改成闭环后终点由"像素误差→0"决定，那些相机参数只影响每步挪多远（收敛快慢），
  // 标不准最多多迭代几次，不影响最终精度。
  double servo_deadband_ratio_;       // 归一化像素误差进死区即算对好
  double servo_gain_;                 // 每步走偏移量的比例，<1 防超调
  double servo_max_step_dm_;          // 单步位移上限，防止误检把飞机拽飞
  double servo_timeout_sec_;          // 超时兜底：不再等对中，直接降高抛包
  double servo_data_timeout_sec_;     // 多久没收到伺服误差算"丢目标"
  // 火源坐标的 UDP 丢包冗余不在本包：见 fire_link_pkg 的 report_repeat 参数

  double dist_sample_min_dm_;         // 里程累计的最小采样步长，滤位姿抖动

  // ── 航点 / 阶段 ──
  std::vector<FireWaypoint> waypoints_;
  MissionPhase phase_;
  std::size_t  current_idx_;
  FireWaypoint fire_wp_;              // 发挥阶段的临时目标（火源上方）
  bool         use_fire_wp_;          // true 时 publishTarget 发 fire_wp_ 而非 waypoints_[idx]
  std::size_t  patrol_resume_idx_;    // 火情打断时的巡逻航点索引

  // 火源状态
  bool          fire_handled_;        // 已抛包+已报坐标，之后不再响应检测
  bool          fire_confirmed_;      // 位置已稳定，锁定坐标
  bool          fire_reported_;       // 已向消防车报过坐标（确认时先报一次，抛完再报一次）
  bool          has_fire_position_;
  double        fire_x_dm_;
  double        fire_y_dm_;
  double        fire_candidate_x_dm_; // 正在做稳定性判定的候选位置
  double        fire_candidate_y_dm_;
  rclcpp::Time  fire_candidate_since_;
  bool          latest_fire_detected_;

  // 视觉伺服最新一帧误差（/fire/servo_error）
  bool          has_servo_error_;
  double        servo_ex_;            // 归一化像素误差，+ = 画面右
  double        servo_ey_;            // 归一化像素误差，+ = 画面下
  double        servo_bx_cm_;         // 机体系前向偏移
  double        servo_by_cm_;         // 机体系左向偏移
  rclcpp::Time  servo_error_time_;
  bool          servo_timed_out_;     // 超时兜底已触发（照常降高抛包）

  // 阶段计时
  rclcpp::Time  phase_start_time_;
  bool          drop_sent_;
  rclcpp::Time  drop_sent_time_;

  // 里程
  double        patrol_distance_dm_;
  bool          has_last_sample_;
  double        last_sample_x_dm_;
  double        last_sample_y_dm_;

  // ── ROS ──
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr target_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr             active_controller_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr             route_choice_pub_;
  // 机腹激光笔：香橙派 40pin(PB0) + WiringOP `gpio` 命令，由 magnet_control_pkg 的
  // Python 节点实际控脚。这里只发命令字（1=on / 2=off）。
  // ⚠ 不要再走 /electromagnet_control(飞控 0x33)——盘点题已实测不稳并弃用。
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr             laser_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr             alarm_led_pub_;       // 机上报警 LED
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr             drop_pub_;            // 抛洒灭火包
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr drone_pose_pub_;      // →消防车 1Hz
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr patrol_distance_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr fire_report_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr            fire_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr             mission_complete_pub_;

  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr              start_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr              height_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr               fire_detected_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr  fire_position_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr  servo_error_sub_;

  rclcpp::TimerBase::SharedPtr  monitor_timer_;
  rclcpp::TimerBase::SharedPtr  telemetry_timer_;

  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  mutable std::mutex mutex_;
  bool   has_height_;
  double current_height_dm_;
  bool   mission_complete_sent_;
  bool   first_publish_done_;

  std::string  last_status_text_;
  rclcpp::Time last_status_pub_time_;
  bool         status_inited_;
};

}  // namespace fire_control_pkg
