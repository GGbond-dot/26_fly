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

// ───────── 物理标定常量（编译期宏，改后需重新编译）─────────
// 标定办法：飞机静止贴地，分别读两个激光的输出，把读数填进下面。
// 注意：这三个是宏、不是 ROS 参数，只能改这里再 colcon build。

// 面阵激光贴地读数（cm）。也用于下降目标高度换算：z = 柱高 + 此值 + 余隙。
#ifndef LASER_AREA_BASE_CM
#define LASER_AREA_BASE_CM   13.0   // TODO: 实测填入
#endif
// 点阵激光贴地读数（cm）。
#ifndef LASER_POINT_BASE_CM
#define LASER_POINT_BASE_CM   4.0   // TODO: 实测填入
#endif
// 两激光的 z 向安装高度差（cm）= 面阵 - 点阵（面阵装得高）。
// 柱高反推用的修正量：柱高 = (面阵地面 - 此差值) - 点阵读数。
// 默认由上面两个贴地读数相减得到；若你直接量了差值，在此覆盖即可。
#ifndef LASER_MOUNT_DIFF_CM
#define LASER_MOUNT_DIFF_CM  (LASER_AREA_BASE_CM - LASER_POINT_BASE_CM)
#endif

// ★ R：机械臂伸直且臂尖触地时的【面阵激光读数】(cm)。第二趟下降的唯一高度参照。
// 要让臂尖落到柱顶(z=柱高)，面阵目标读数 = R + 柱高。
// 标定法：飞机悬停到"机械臂伸直、臂尖刚触地"，读 /laser_array/ground_height 即为 R。
// （此刻点阵被伸出的机械臂挡住=垃圾值，正好印证第二趟不能用点阵。）
// 一个数直接量出臂长，取代旧式 LASER_AREA_BASE_CM + grab_clearance 的拼凑。
#ifndef ARM_GROUND_AREA_CM
#define ARM_GROUND_AREA_CM  27.0   // 实测标定 2026-05-22（臂尖触地稳定段面阵中位数27/均值27.23/σ0.49，220445.csv）
#endif

// 摄像头相对点阵激光的 xy 物理偏置（cm，机体系）——待测。
// 视觉把柱子对到相机中心后，机体再按此偏置平移，让点阵/机械臂正对柱子。
// 提示：运行期实际用的是 launch 里的 cam_offset_dx_cm / cam_offset_dy_cm 参数，
//       下面这俩宏只是参数缺省值的兜底。要改优先改 launch（不用重编）。
#ifndef CAM_TO_POINT_LASER_DX_CM
#define CAM_TO_POINT_LASER_DX_CM  0.0   // TODO: 实测填入
#endif
#ifndef CAM_TO_POINT_LASER_DY_CM
#define CAM_TO_POINT_LASER_DY_CM  0.0   // TODO: 实测填入
#endif

// 摄像头相对机械臂吸取点的 xy 物理偏置（cm，机体系）。
// 2026-05-22：机械结构已改——舵机机械臂伸直后吸取点正好落在摄像头光轴竖直线上，
// 故吸取点 xy = 相机中心 xy，此偏置物理真值就是 0/0（不再是待标定占位）。
// 含义：第二趟铁片对到画面中心即可竖直下降取物，DESCEND 不需平移。
// 与 cam_offset(相机→点阵激光) 仍是两个偏置：cam_offset 测高那步把激光移到柱心。
// 若实飞发现伸臂后吸取点与相机有微小残差，可在 launch 里给 arm_offset 填小量微调。
#ifndef CAM_TO_ARM_DX_CM
#define CAM_TO_ARM_DX_CM  0.0   // 机械共轴 → 0
#endif
#ifndef CAM_TO_ARM_DY_CM
#define CAM_TO_ARM_DY_CM  0.0   // 机械共轴 → 0
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

// 第一趟（SURVEY）对每根柱子的测量结果
struct PillarSurvey
{
  double x_cm = 0.0;
  double y_cm = 0.0;
  double height_cm = 0.0;      // 点阵反推的柱顶高度
  bool   has_height = false;
  double plate_ratio = 0.0;    // 圆/方框面积占比（越大铁片越大；与飞行高度无关）
  bool   has_plate = false;    // 该柱顶是否检出铁片
};

enum class MissionPhase
{
  SCAN,          // 起飞 → 扫描终点，沿途累积柱子检测
  WAIT_PILLARS,  // 等 /detected_pillars 聚类结果
  SURVEY,        // 第一趟：逐柱测高 + 读铁片占比（不下降不抓取）
  PICKUP,        // 第二趟：按占比大→小抓取，叠到空柱
  LAND,          // 降落到对角起停区
  DONE
};

// 第一趟每根柱子的子状态
enum class SurveySub
{
  APPROACH,        // 飞到柱子上方巡航高度
  CENTER,          // 视觉对准铁片/边框中心 + 采集铁片占比 + 找到铁片声光滞空
  MEASURE_HEIGHT   // +cam_offset 把点阵移到柱心测柱高并存储（机械臂收起、点阵无遮挡；第二趟复用）
};

// 第二趟每片铁片的子状态
enum class PickupSub
{
  APPROACH,         // 飞到目标柱上方巡航高度
  CENTER,           // 视觉对准铁片中心（接管，xy 精对）
  MEASURE_HEIGHT_GRAB, // 抓取下降前现场测当前柱高；失败回退第一趟柱高
  DESCEND_MID,      // 下降到下一个中停读数（分段下降；进入第一段即开电磁铁早充磁）
  RECENTER_MID,     // 中停高度二次视觉对准（分段对准，带超时防卡死）
  DESCEND_FINAL,    // 盲降到 R + 柱高 − grab_press（最后一段，压住吸取）
  HOVER_GRAB,       // 机械臂伸出 + 吸磁，悬停
  CLIMB_BACK,       // 爬回巡航高度
  OBSERVE_GRAB,     // 观察 /circle_area_ratio 判抓取成败 / 重试
  GOTO_DROP,        // 飞到空柱上方
  CENTER_DROP,      // 视觉对准空柱边框中心（接管，仅 xy 精对；空柱高第一趟已测）
  DESCEND_DROP,     // 下降到 R + 空柱高 + 已叠高度 + drop_gap
  HOVER_DROP,       // 机械臂伸出 + 松磁，悬停
  CLIMB_AFTER_DROP  // 爬回巡航高度
};

// 降落（对角起停区 B）的子状态：视觉对准 B 黑框中心后分段精准降落
enum class LandSub
{
  APPROACH,    // 飞到 B 上方对准高度（粗定位）
  CENTER,      // 视觉接管对准 B 黑框中心，对上后固化降落 anchor
  DESCEND_MID, // 关接管，降 land_recenter_drop_cm 到中停高度（位置保持在 anchor）
  RECENTER,    // 中停高度再视觉对准一次 B 框，更新 anchor（近一截、像素更准）
  DESCEND      // 关接管，竖直降到地面（位置保持在 anchor）
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
  void publishServo(bool extended);   // true=机械臂伸出, false=收起（串口帧 0x11）
  void publishMagnet(bool on);        // true=吸, false=松（串口帧 0x33）
  void publishBuzzerLed(bool on);     // true=蜂鸣器+LED开, false=关（串口帧 0x22）

  // ── 阶段步进 ──
  void stepWaitPillars(double x_cm, double y_cm);
  void stepWaypoints(double x_cm, double y_cm, double z_cm, double yaw_deg);  // SCAN / LAND
  void enterSurveySub(SurveySub s);
  void stepSurvey(double x_cm, double y_cm, double z_cm);
  void finishSurvey();
  void planPickupOrder();
  void enterPickupSub(PickupSub s);
  void stepPickup(double x_cm, double y_cm, double z_cm);
  void enterLandSub(LandSub s);
  void stepLanding(double x_cm, double y_cm, double z_cm);  // 视觉对准 B 框精准降落
  void buildDescendPlan(double pillar_height_cm, bool is_drop);  // 按柱高生成分段中停读数表
  void startGrabDescend();                         // 建表后进入下降（有中停→DESCEND_MID，否则直接 FINAL）
  void startDropDescend();                         // 放置侧复用分段下降/中停对准
  void startLanding();

  // ── 工具 ──
  std::vector<std::size_t> greedyOrderIdx(
    const std::vector<std::pair<double, double>> & pts,
    double start_x_cm, double start_y_cm) const;
  std::size_t nearestToLanding() const;

  bool isVisuallyAligned() const;
  void recordFineData(int dx_px, int dy_px);
  bool shouldSkipGrabVisual() const;
  void getDropTargetXY(double & x_cm, double & y_cm) const;
  bool dropVisionCircleVeto() const;
  void updateDropAnchorFromVision(double x_cm, double y_cm, const char * reason);

  // 现场测柱高：命中帧(drop≥阈值)分簇，取“最高的≥min_cluster_frames 簇”的中位数为柱顶（柱顶给最大drop）
  bool tryStepHeight(double & out_cm);
  bool finalizePlateRatio(double & out_ratio) const;

  void applyCameraOffsetToTarget(double & x_cm, double & y_cm, double yaw_deg) const;
  void applyArmOffsetToTarget(double & x_cm, double & y_cm, double yaw_deg) const;

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

  // 精准降落（视觉对准 B 黑框中心）
  bool   land_visual_enable_;       // true=飞到 B 上方先视觉对准框中心再降；false=纯位置降落（旧行为）
  double land_align_height_cm_;      // 对准 B 框时的悬停高度（要能看全 50cm 框）
  double land_recenter_drop_cm_;     // 第一次对准后再下降此高度，到中停高度二次对准一次（默认60）

  // 视觉对准
  double visual_align1_timeout_sec_;
  double visual_align2_timeout_sec_;
  int    visual_pixel_tol_;
  int    visual_align_required_hits_;
  int    visual_jump_px_;
  double visual_stale_sec_;
  double cam_offset_dx_cm_;   // 相机→点阵激光（测高用）
  double cam_offset_dy_cm_;
  double arm_offset_dx_cm_;   // 相机→机械臂吸取点（下降取物/放置用）
  double arm_offset_dy_cm_;

  // 高度采样：sample_pillar_drop_thresh_cm_ 复用为“点阵突变”阈值
  double sample_duration_sec_;
  int    sample_min_pillar_frames_;
  double sample_pillar_drop_thresh_cm_;

  // 第二趟现场测柱高（命中帧分簇，取“最高的密集簇”=真柱顶，不被柱沿低值拉偏）
  double measure_height_timeout_sec_;     // 测高超时 → 放弃该片，不下降
  int    height_min_hit_frames_;          // 累计多少帧点阵打中柱顶(drop≥阈值)才开始分簇出结果
  double height_cluster_gap_cm_;          // 分簇间隔：排序后相邻 drop 间隔 > 此值即切簇（默认8）
  int    height_min_cluster_frames_;      // 一个簇至少多少帧才可信（滤单帧毛刺，默认3）
  double live_height_consistency_cm_;     // 现场重测与第一趟柱高差 > 此值 → 判现场不可信，回退第一趟（默认15）

  // 铁片占比采样（判大小）
  int    plate_min_ratio_frames_;   // 合格占比样本最少帧数（少于则判为空柱）

  // 第一趟找到铁片后的声光提示 + 滞空时长（题目发挥要求："找到滞空3s+声光提示"）
  double survey_signal_hold_sec_;   // 占比帧够即开声光，悬停此时长后关声光再飞下一柱

  // 下降 / 抓取 / 叠放（z 全是面阵目标读数；抓取/放置用 R=ARM_GROUND_AREA_CM 参照）
  double mid_clearance_cm_;   // （已废弃，抓取改用分段下降；保留声明避免参数报错）
  double descend_seg_len_cm_;       // 分段下降：每个中停段下降量（默认 30）
  double descend_min_tail_cm_;      // 分段下降：最后盲降段下限，剩余 < seg+tail 不再分段（默认 20）
  double descend_recenter_timeout_sec_; // 中停二次对准超时（防卡死，默认 2.5）
  double descend_abort_area_cm_; // 下降安全底线：面阵读数 < 此值(臂尖将到地面以下=不可能)立即中止下降爬回重试（默认22≈R-5）
  double descend_seg_len_min_plate_cm_; // 抓最小片时用更短分段（默认20），放置不加密
  double grab_press_cm_;      // 抓取下压量：z = R + 柱高 − grab_press（压住铁片确保吸牢）
  double drop_gap_cm_;        // （放置已改"上方释放"，不再使用；保留避免参数报错）
  double drop_press_cm_;      // （放置已改"上方释放"，不再使用；保留避免参数报错）
  double drop_release_clearance_cm_;   // 放置末段不贴死：z = R + 空柱高 + 已叠 + 此余隙（叠面上方释放）
  double drop_post_release_hover_sec_; // 放置松磁后原地悬停时长（防机体惯性带偏），取代旧 drop_settle 语义
  double grab_final_dy_cm_;   // 抓取末段盲降额外 y 偏置（放置不用）
  double drop_final_dy_cm_;    // 放置末段额外 y 偏置：补电磁铁吸取点物理偏置，防铁片偏左滚落（与 grab 同向，map +y=画面左）
  double drop_final_dx_cm_;    // 放置末段额外 x 偏置（map +x=画面正上方）：放置专用，正值往前补
  // 抓取下降模式 A/B：segmented=分段中停二次对准；direct_after_center=对准中心后直接盲降到位（不分段）
  std::string grab_descend_mode_;
  double arm_extend_sec_;     // 放置时机械臂伸直到位耗时
  double drop_settle_sec_;    // （放置已改用 drop_post_release_hover_sec；保留避免参数报错）
  double hover_grab_sec_;
  double plate_thickness_cm_;        // 每片铁片厚度，用于叠放时逐层抬高落点
  bool   skip_largest_grab_visual_align_; // 最大铁片抓取不做视觉微调，避免过正导致气动抖动
  double empty_pillar_side_cm_;           // 空柱大柱边长，用于限制视觉 anchor 相对点云中心的最大可信修正
  bool   drop_visual_anchor_enable_;      // 空柱放置：视觉确认后记录/复用真实放置 anchor
  double drop_anchor_max_correction_cm_;  // 首次 anchor 相对点云空柱坐标的最大允许修正
  double drop_anchor_max_update_step_cm_; // 后续 anchor 单次更新最大步长，防止视觉/点云异常带偏
  double drop_visual_circle_veto_sec_;    // 已叠放后近期看到圆盘则不信空柱视觉更新
  bool   traverse_only_mode_;        // true: 只做第一趟（占比）然后降落（试飞用，不测高不抓取）
  bool   measure_only_mode_;         // true: 跑到第二趟测高即止，逐片测完高就跳过，不下降不抓（验证测高用）
  double target_republish_period_sec_;

  // 抓取观察 + 重试
  double pickup_check_observe_sec_;
  int    pickup_max_attempts_;
  int    pickup_observe_plate_frames_required_;

  // ── 状态 ──
  MissionPhase phase_;
  std::vector<PickupWaypoint> waypoints_;   // SCAN + LAND 用
  std::size_t scan_end_idx_;
  std::size_t current_idx_;

  // 检测到的柱子（map 系 cm）
  std::vector<std::pair<double, double>> detected_pillars_cm_;
  bool pillars_received_;
  rclcpp::Time wait_pillars_start_time_;

  // 第一趟
  std::vector<PillarSurvey> survey_results_;   // 与 detected_pillars_cm_ 同序、同索引
  std::vector<std::size_t>  survey_order_;     // 访问顺序（就近遍历），值为 detected 索引
  std::size_t survey_iter_;
  SurveySub   survey_sub_;
  bool        survey_collecting_ratio_;        // ALIGN 期间是否在采集 /circle_area_ratio
  std::vector<double> plate_ratio_samples_;
  bool        survey_signal_active_ = false;   // 当前柱是否已"找到铁片"并开了声光
  rclcpp::Time survey_signal_start_;           // 声光开启(=找到铁片)时刻，用于计 3s 滞空

  // 空柱（放置目标）= 离 landing(B) 最近的那根
  std::size_t empty_idx_;
  double empty_pillar_x_cm_ = 0.0;
  double empty_pillar_y_cm_ = 0.0;
  double empty_pillar_height_cm_ = 0.0;
  bool   has_empty_pillar_height_ = false;
  bool   has_drop_anchor_ = false;
  double drop_anchor_x_cm_ = 0.0;
  double drop_anchor_y_cm_ = 0.0;

  // 降落 anchor：视觉对准 B 框时机体就在框中心正上方，记其实际 map 位姿，竖直降到此处
  LandSub land_sub_ = LandSub::APPROACH;
  bool    has_land_anchor_ = false;
  double  land_anchor_x_cm_ = 0.0;
  double  land_anchor_y_cm_ = 0.0;

  // 第二趟
  std::vector<std::size_t> pickup_order_;      // 待抓铁片柱索引，按占比大→小
  std::size_t pickup_iter_;
  PickupSub   pickup_sub_;
  int         stack_count_ = 0;                // 已叠到空柱上的铁片数
  bool        carrying_plate_ = false;
  int         pickup_attempts_ = 0;
  bool        pickup_observed_plate_ = false;
  int         pickup_observed_plate_frames_ = 0;
  double      pickup_live_height_cm_ = 0.0;
  bool        has_pickup_live_height_ = false;

  // 抓取分段下降：中停读数表（从高到低）+ 当前段索引
  std::vector<double> descend_checkpoints_;
  std::size_t         descend_ckpt_idx_ = 0;
  bool                descend_is_drop_ = false;
  bool                drop_released_ = false;

  // 子阶段目标（内存驻留，供重发）
  PickupWaypoint sub_target_;
  rclcpp::Time sub_enter_time_;
  rclcpp::Time sub_hover_start_;

  // 视觉对准
  std::deque<std::pair<int, int>> fine_hist_;
  rclcpp::Time last_fine_time_;
  bool has_fine_;
  int  last_fine_dx_;
  int  last_fine_dy_;
  bool visual_takeover_active_;

  // 高度采样
  std::vector<double> pillar_height_samples_cm_;
  rclcpp::Time sample_start_time_;
  std::vector<double> height_hit_samples_; // 测高：窗口内所有打中柱顶(drop≥阈值)的帧，分簇取最高密集簇

  // visual_pkg 的铁片面积占比
  double last_circle_ratio_ = 0.0;
  bool   has_circle_ratio_  = false;
  rclcpp::Time last_circle_valid_time_;

  // SCAN/LAND 航点悬停计时
  bool is_hovering_;
  rclcpp::Time hover_start_time_;

  // 激光数据
  bool   has_area_height_;
  double area_height_cm_;
  bool   has_point_height_;
  double point_height_cm_;

  // 目标重发
  PickupWaypoint last_target_{};
  bool republish_enabled_ = false;

  bool mission_complete_sent_;

  // ── ROS ──
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr    target_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                 enable_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                 visual_takeover_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                 visual_takeover_active_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr                active_controller_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr                route_choice_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr                servo_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr                electromagnet_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr                buzzer_led_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr                mission_complete_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr                pickup_done_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr                pickup_failed_pub_;

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
