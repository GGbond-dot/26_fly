#pragma once

#include <cstddef>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace activity_control_pkg
{

// 两个激光在机身上的物理安装高度（cm）——静止在地面时各自的读数
#ifndef LASER_AREA_BASE_CM
#define LASER_AREA_BASE_CM   13.0
#endif
#ifndef LASER_POINT_BASE_CM
#define LASER_POINT_BASE_CM   4.0
#endif
// 两激光发射点的 z 向固定差值（面阵高于点阵）
#ifndef LASER_MOUNT_DIFF_CM
#define LASER_MOUNT_DIFF_CM  (LASER_AREA_BASE_CM - LASER_POINT_BASE_CM)
#endif

// 摄像头相对点阵激光的 xy 物理偏置（cm，机体系）——待测
#ifndef CAM_TO_POINT_LASER_DX_CM
#define CAM_TO_POINT_LASER_DX_CM  0.0
#endif
#ifndef CAM_TO_POINT_LASER_DY_CM
#define CAM_TO_POINT_LASER_DY_CM  0.0
#endif

struct PickupWaypoint
{
  double x_cm;
  double y_cm;
  double z_cm;
  double yaw_deg;
  double hover_sec;
  const char * tag;
};

enum class PickupPhase
{
  SCAN,
  WAIT_PILLARS,
  PICKUP,    // 访问每根柱子：对准 → 测高 → 下降 → 抓取(预留) → 爬升
  LAND,
  DONE
};

enum class PickupSub
{
  APPROACH,           // 飞到 (px, py, 150) 上方
  ALIGN1,             // 视觉对准 + 双激光采样
  DESCEND_MID,        // 面阵下降到 pillar_top + 30cm
  ALIGN2,             // 近距短视觉对准 (best-effort)
  DESCEND_FINAL,      // 盲降到 pillar_top + Δgrab（= 铁片上方 20cm）
  HOVER_GRAB,         // 到位发抓取信号给飞控 + 吸磁，悬停
  CLIMB_BACK,         // 爬回巡航高度
  GOTO_DROP,          // 飞到空柱子 xy，巡航高度
  DESCEND_DROP,       // 下降到空柱上方 20cm
  HOVER_DROP,         // 到位发放置信号给飞控 + 松磁，悬停
  CLIMB_AFTER_DROP    // 爬回巡航高度
};

class PillarPickupMissionNode : public rclcpp::Node
{
public:
  explicit PillarPickupMissionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ── 回调 ──
  void areaHeightCallback(const std_msgs::msg::Int16::SharedPtr msg);
  void pointHeightCallback(const std_msgs::msg::Int16::SharedPtr msg);
  void pillarsCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  void fineDataCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
  void circleRatioCallback(const std_msgs::msg::Float32::SharedPtr msg);
  void monitorTimerCallback();

  // ── 姿态 / 到位判定 ──
  bool getCurrentPose(double & x_cm, double & y_cm, double & yaw_deg);
  bool isReached(const PickupWaypoint & wp, double x_cm, double y_cm,
                 double z_cm, double yaw_deg) const;

  // ── 发布 ──
  void publishTarget(const PickupWaypoint & wp, bool verbose = true);
  void publishEnable(bool on);
  void publishVisualTakeover(bool on);
  void publishActiveController(uint8_t mode);
  // 通过 uart_to_stm32 的 /arm_control 通道告诉飞控机械臂动作：
  // 1 = 伸出，0 = 收回。
  void publishArmAction(uint8_t code);
  // 控制电磁铁：on=true 吸住，on=false 松开。magnet_control 节点 IO 低电平有效。
  void publishMagnet(bool on);

  // ── SCAN / LAND 路线构造 ──
  std::vector<std::pair<double, double>> greedyOrder(
    const std::vector<std::pair<double, double>> & pillars,
    double start_x_cm,
    double start_y_cm) const;

  // ── 每柱子状态机 ──
  void enterSub(PickupSub s);
  void stepPickup(double x_cm, double y_cm, double z_cm, double yaw_deg);

  // 视觉对准判定：连续 K 帧 |dx|,|dy| <= tol
  bool isVisuallyAligned() const;
  // 记录一帧 fine_data，用于连续对准 / 跳变检测
  void recordFineData(int dx_px, int dy_px);

  // 高度采样：每帧 (h_area, h_point) 判定是否"点阵打到柱子"，累积样本
  void tryAccumulateHeightSample();
  // 从已累积样本计算 h_pillar；成功返回 true 并写入 out
  bool finalizePillarHeight(double & out_cm) const;

  // 把视觉像素偏置转成 xy 物理偏置补偿（给目标位置）
  void applyCameraOffsetToTarget(double & x_cm, double & y_cm, double yaw_deg) const;

  static double meterToCm(double v) { return v * 100.0; }
  static double radToDeg(double v);
  double normalizeAngleDeg(double angle_deg) const;
  static double degToRad(double v) { return v * M_PI / 180.0; }

  // ── 参数 ──
  std::string map_frame_;
  std::string laser_link_frame_;

  double pos_tol_cm_;
  double yaw_tol_deg_;
  double height_tol_cm_;

  double flight_height_cm_;
  double land_height_cm_;
  double scan_end_x_cm_;
  double landing_x_cm_;
  double landing_y_cm_;
  double pillar_visit_height_cm_;
  double pillar_wait_timeout_sec_;

  // 视觉对准
  double visual_align1_timeout_sec_;
  double visual_align2_timeout_sec_;
  int    visual_pixel_tol_;          // |dx|,|dy| 像素容差
  int    visual_align_required_hits_; // 连续满足帧数
  int    visual_jump_px_;            // 相邻两帧跳变阈值
  double visual_stale_sec_;          // fine_data 失效时间
  double cam_offset_dx_cm_;          // 摄像头-点阵激光 xy 偏置（运行期可覆盖宏默认）
  double cam_offset_dy_cm_;

  // 高度采样
  double sample_duration_sec_;        // 采样最长时间
  int    sample_min_pillar_frames_;   // 合格柱子帧最少数
  double sample_pillar_drop_thresh_cm_; // h_point 比预期地面读数小多少才判为"柱子帧"

  // 下降 / 抓取
  double mid_clearance_cm_;           // DESCEND_MID 目标: pillar_top + 此值（对应面阵读数附加项）
  double grab_clearance_cm_;          // DESCEND_FINAL 目标: pillar_top + 此值
  double hover_grab_sec_;             // 抓取/放置占位悬停时间
  bool   traverse_only_mode_;         // true: 仅遍历柱子，完成后直接去终点
  double target_republish_period_sec_; // 目标重发周期（秒），用于跨节点启动时序抖动

  // ── 状态 ──
  PickupPhase phase_;
  std::vector<PickupWaypoint> waypoints_;   // SCAN + LAND 用
  std::size_t scan_end_idx_;
  std::size_t current_idx_;

  // 柱子访问队列
  std::vector<std::pair<double, double>> pillars_ordered_cm_;
  std::size_t pillar_iter_;
  PickupSub sub_;
  PickupWaypoint sub_target_;               // 当前子阶段的目标航点（内存驻留）
  rclcpp::Time sub_enter_time_;
  bool sub_is_hovering_;
  rclcpp::Time sub_hover_start_;

  // 视觉对准
  std::deque<std::pair<int, int>> fine_hist_;  // 最近 N 帧 fine_data
  rclcpp::Time last_fine_time_;
  bool has_fine_;
  int  last_fine_dx_;
  int  last_fine_dy_;
  bool visual_takeover_active_;

  // 高度采样
  std::vector<double> pillar_height_samples_cm_;
  rclcpp::Time sample_start_time_;
  double measured_pillar_height_cm_;
  bool   has_measured_pillar_height_;

  // 柱子检测
  std::vector<std::pair<double, double>> detected_pillars_cm_;
  bool pillars_received_;
  rclcpp::Time wait_pillars_start_time_;

  // 空柱子（距 landing 点 B 最近的那根，作为放置目标）
  double empty_pillar_x_cm_ = 0.0;
  double empty_pillar_y_cm_ = 0.0;
  bool   has_empty_pillar_  = false;
  double empty_pillar_height_cm_ = 0.0;
  bool   has_empty_pillar_height_ = false;

  // 是否正在携带铁片（HOVER_GRAB 完成到 HOVER_DROP 完成之间为 true）
  bool carrying_plate_ = false;

  // visual_pkg 的铁片面积占比，仅用于采样期间记录，不影响状态机
  double last_circle_ratio_ = 0.0;
  bool   has_circle_ratio_  = false;

  // 是否在 SCAN 的某段上悬停计时
  bool is_hovering_;
  rclcpp::Time hover_start_time_;

  // 激光数据
  bool   has_area_height_;
  double area_height_cm_;
  bool   has_point_height_;
  double point_height_cm_;

  bool mission_complete_sent_;

  // ── ROS ──
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr    target_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                 enable_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                 visual_takeover_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr                active_controller_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr                route_choice_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr                arm_action_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr                magnet_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr                mission_complete_pub_;

  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr             area_height_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr             point_height_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr pillars_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr   fine_data_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr           circle_ratio_sub_;

  rclcpp::TimerBase::SharedPtr                                      monitor_timer_;
  rclcpp::TimerBase::SharedPtr                                      target_republish_timer_;
  rclcpp::TimerBase::SharedPtr                                      route_choice_kick_timer_;
  int                                                               route_choice_kick_count_ = 0;

  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  mutable std::mutex mutex_;
};

}  // namespace activity_control_pkg
