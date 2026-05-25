#include "spray_control_pkg/spray_mission_node.hpp"

#include <algorithm>
#include <cmath>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace spray_control_pkg
{

using namespace std::chrono_literals;

SprayMissionNode::SprayMissionNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("spray_mission_node", options),
  phase_(MissionPhase::TAKEOFF),
  current_idx_(0),
  spraying_(false),
  blink_done_(0),
  laser_on_(false),
  has_height_(false),
  current_height_cm_(0.0),
  mission_complete_sent_(false)
{
  // ── 参数 ──
  map_frame_   = declare_parameter<std::string>("map_frame", "map");
  base_frame_  = declare_parameter<std::string>("base_frame", "base_link");

  pos_tol_cm_    = declare_parameter<double>("pos_tol_cm", 12.0);
  yaw_tol_deg_   = declare_parameter<double>("yaw_tol_deg", 10.0);
  height_tol_cm_ = declare_parameter<double>("height_tol_cm", 10.0);

  flight_height_cm_ = declare_parameter<double>("flight_height_cm", 150.0);
  land_height_cm_   = declare_parameter<double>("land_height_cm", 0.0);
  home_x_cm_        = declare_parameter<double>("home_x_cm", 0.0);
  home_y_cm_        = declare_parameter<double>("home_y_cm", 0.0);

  spray_blink_count_ = declare_parameter<int>("spray_blink_count", 2);
  spray_on_sec_      = declare_parameter<double>("spray_on_sec", 0.4);
  spray_off_sec_     = declare_parameter<double>("spray_off_sec", 0.4);

  // ── ROS 接口 ──
  auto durable_qos = rclcpp::QoS(10).reliable().transient_local();
  target_pub_            = create_publisher<std_msgs::msg::Float32MultiArray>("/target_position", durable_qos);
  active_controller_pub_ = create_publisher<std_msgs::msg::UInt8>("/active_controller", durable_qos);
  laser_pub_             = create_publisher<std_msgs::msg::UInt8>("/laser_control", durable_qos);
  mission_complete_pub_  = create_publisher<std_msgs::msg::Empty>("/mission_complete", rclcpp::QoS(10).reliable());

  height_sub_ = create_subscription<std_msgs::msg::Int16>(
    declare_parameter<std::string>("height_topic", "/laser_array_ground_height"), rclcpp::SensorDataQoS(),
    std::bind(&SprayMissionNode::heightCallback, this, std::placeholders::_1));

  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  buildCoverageWaypoints();
  publishLaser(false);

  monitor_timer_ = create_wall_timer(
    50ms, std::bind(&SprayMissionNode::monitorTimerCallback, this));

  RCLCPP_INFO(get_logger(), "撒药任务节点已启动，共 %zu 个航点，巡航高度 %.0fcm",
              waypoints_.size(), flight_height_cm_);
}

void SprayMissionNode::heightCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  current_height_cm_ = static_cast<double>(msg->data);
  has_height_ = true;
}

// ─────────────────────────────────────────────────────────────────────────────
// 全覆盖航点生成
//
// TODO(场地标定): green_blocks_ 的真实坐标需按图 1 实测填写。
//   作业区 400cm(Y) × 500cm(X)，50×50cm 区块编号 1~28，绿色为播撒区。
//   起点区块 A = 21。下面给的是占位栅格，仅用于打通状态机流程，
//   实测后用每个绿色区块中心坐标替换，并按蛇形（boustrophedon）顺序排列以避免重复/漏撒。
//   发挥部分(1): 把被改为灰色的 3~4 个连续区块从列表剔除即可。
// ─────────────────────────────────────────────────────────────────────────────
void SprayMissionNode::buildCoverageWaypoints()
{
  waypoints_.clear();

  const double z = flight_height_cm_;

  // 1) 起飞：原地升到巡航高度
  waypoints_.push_back({home_x_cm_, home_y_cm_, z, 0.0, false, 0, "takeoff"});

  // 2) 飞往起点区块 A(21) —— 占位坐标，需实测替换
  const double a_x = declare_parameter<double>("block_a_x_cm", 75.0);
  const double a_y = declare_parameter<double>("block_a_y_cm", 350.0);
  waypoints_.push_back({a_x, a_y, z, 0.0, true, 21, "block_A"});

  // 3) 覆盖路径占位：从 green_blocks 参数读 [id,x,y, id,x,y, ...]
  //    没配置时退化为只撒起点区块，保证骨架可编译可跑通流程。
  std::vector<double> blocks =
    declare_parameter<std::vector<double>>("green_blocks", std::vector<double>{});
  for (std::size_t i = 0; i + 2 < blocks.size(); i += 3) {
    SprayWaypoint wp;
    wp.block_id = static_cast<int>(blocks[i]);
    wp.x_cm     = blocks[i + 1];
    wp.y_cm     = blocks[i + 2];
    wp.z_cm     = z;
    wp.yaw_deg  = 0.0;
    wp.spray    = true;
    wp.tag      = "block";
    waypoints_.push_back(wp);
  }

  // 4) 返航到起降点上空
  waypoints_.push_back({home_x_cm_, home_y_cm_, z, 0.0, false, 0, "return"});

  // 5) 降落
  waypoints_.push_back({home_x_cm_, home_y_cm_, land_height_cm_, 0.0, false, 0, "land"});
}

bool SprayMissionNode::getCurrentPose(double & x_cm, double & y_cm, double & yaw_deg)
{
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "tf 查询失败: %s", ex.what());
    return false;
  }
  x_cm = meterToCm(tf.transform.translation.x);
  y_cm = meterToCm(tf.transform.translation.y);
  yaw_deg = normalizeAngleDeg(tf2::getYaw(tf.transform.rotation) * 180.0 / M_PI);
  return true;
}

bool SprayMissionNode::isReached(const SprayWaypoint & wp, double x_cm, double y_cm,
                                 double z_cm, double yaw_deg) const
{
  const double dxy = std::hypot(wp.x_cm - x_cm, wp.y_cm - y_cm);
  const double dz  = std::fabs(wp.z_cm - z_cm);
  const double dyaw = std::fabs(normalizeAngleDeg(wp.yaw_deg - yaw_deg));
  return dxy <= pos_tol_cm_ && dz <= height_tol_cm_ && dyaw <= yaw_tol_deg_;
}

void SprayMissionNode::publishTarget(const SprayWaypoint & wp)
{
  std_msgs::msg::Float32MultiArray msg;
  msg.data = {static_cast<float>(wp.x_cm), static_cast<float>(wp.y_cm),
              static_cast<float>(wp.z_cm), static_cast<float>(wp.yaw_deg)};
  target_pub_->publish(msg);

  std_msgs::msg::UInt8 active_msg;
  active_msg.data = 2;  // 位置控制器接管
  active_controller_pub_->publish(active_msg);
}

void SprayMissionNode::publishLaser(bool on)
{
  std_msgs::msg::UInt8 msg;
  msg.data = on ? 1 : 0;
  laser_pub_->publish(msg);
  laser_on_ = on;
}

// 在 spray 航点悬停时调用：按 spray_blink_count_ 次闪烁激光笔。
// 返回 true 表示撒药完成。
bool SprayMissionNode::runSpraySequence()
{
  const rclcpp::Time now = this->now();
  if (!spraying_) {
    spraying_ = true;
    blink_done_ = 0;
    publishLaser(true);
    blink_edge_time_ = now;
    return false;
  }

  const double elapsed = (now - blink_edge_time_).seconds();
  if (laser_on_) {
    if (elapsed >= spray_on_sec_) {
      publishLaser(false);
      blink_edge_time_ = now;
      ++blink_done_;
    }
  } else {
    if (blink_done_ >= spray_blink_count_) {
      spraying_ = false;
      return true;
    }
    if (elapsed >= spray_off_sec_) {
      publishLaser(true);
      blink_edge_time_ = now;
    }
  }
  return false;
}

void SprayMissionNode::advance()
{
  if (current_idx_ + 1 < waypoints_.size()) {
    ++current_idx_;
    publishTarget(waypoints_[current_idx_]);
    const auto & wp = waypoints_[current_idx_];
    RCLCPP_INFO(get_logger(), "→ 航点 %zu [%s] block=%d x=%.1f y=%.1f z=%.1f spray=%d",
                current_idx_, wp.tag, wp.block_id, wp.x_cm, wp.y_cm, wp.z_cm, wp.spray);
  } else {
    if (!mission_complete_sent_) {
      mission_complete_pub_->publish(std_msgs::msg::Empty());
      std_msgs::msg::UInt8 stop; stop.data = 0;
      active_controller_pub_->publish(stop);
      publishLaser(false);
      mission_complete_sent_ = true;
      phase_ = MissionPhase::DONE;
      RCLCPP_INFO(get_logger(), "撒药任务完成，已发布 /mission_complete");
    }
  }
}

void SprayMissionNode::monitorTimerCallback()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (phase_ == MissionPhase::DONE) return;
  if (!has_height_) return;

  double x, y, yaw;
  if (!getCurrentPose(x, y, yaw)) return;

  // 首次发布当前航点
  static bool first = true;
  if (first) {
    publishTarget(waypoints_[current_idx_]);
    first = false;
    return;
  }

  const SprayWaypoint & wp = waypoints_[current_idx_];
  if (!isReached(wp, x, y, current_height_cm_, yaw)) return;

  // 到点：若需撒药，先完成闪烁再切下一点
  if (wp.spray && !runSpraySequence()) return;

  advance();
}

double SprayMissionNode::normalizeAngleDeg(double angle_deg) const
{
  while (angle_deg > 180.0)  angle_deg -= 360.0;
  while (angle_deg < -180.0) angle_deg += 360.0;
  return angle_deg;
}

}  // namespace spray_control_pkg
