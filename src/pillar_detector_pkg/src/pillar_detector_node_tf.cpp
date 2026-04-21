#include "pillar_detector_pkg/pillar_detector_node_tf.hpp"

#include <algorithm>
#include <cmath>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace pillar_detector_pkg
{

PillarDetectorTFNode::PillarDetectorTFNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("pillar_detector_tf", options),
  enabled_(false),
  result_published_(false),
  frames_accumulated_(0)
{
  scan_topic_        = declare_parameter<std::string>("scan_topic",        "/scan");
  enable_topic_      = declare_parameter<std::string>("enable_topic",      "/pillar_detect_enable");
  map_frame_         = declare_parameter<std::string>("map_frame",         "map");
  laser_link_frame_  = declare_parameter<std::string>("laser_link_frame",  "laser_link");

  // 地图系柱子有效 bbox（起飞点 arena(25,25) → map(0,0)，变换后）
  map_x_min_m_  = declare_parameter("map_x_min_m",  0.25);
  map_x_max_m_  = declare_parameter("map_x_max_m",  2.25);
  map_y_min_m_  = declare_parameter("map_y_min_m", -2.25);
  map_y_max_m_  = declare_parameter("map_y_max_m", -0.25);

  group_dist_m_            = declare_parameter("group_dist_m",            0.25);
  min_pts_per_group_       = declare_parameter("min_pts_per_group",       4);
  min_pillar_separation_m_ = declare_parameter("min_pillar_separation_m", 0.40);

  cluster_merge_dist_m_ = declare_parameter("cluster_merge_dist_m", 0.20);
  min_votes_            = declare_parameter("min_votes",            8);
  max_pillars_          = declare_parameter("max_pillars",          4);

  tf_timeout_sec_ = declare_parameter("tf_timeout_sec", 0.05);

  tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, rclcpp::SensorDataQoS(),
    std::bind(&PillarDetectorTFNode::scanCallback, this, std::placeholders::_1));

  auto enable_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  enable_sub_ = create_subscription<std_msgs::msg::Bool>(
    enable_topic_, enable_qos,
    std::bind(&PillarDetectorTFNode::enableCallback, this, std::placeholders::_1));

  pillars_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
    "/detected_pillars",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  RCLCPP_INFO(get_logger(),
    "TF 柱子检测启动: scan='%s' enable='%s' frames=[%s <- %s]",
    scan_topic_.c_str(), enable_topic_.c_str(), map_frame_.c_str(), laser_link_frame_.c_str());
  RCLCPP_INFO(get_logger(),
    "世界 bbox: x=[%.2f, %.2f] y=[%.2f, %.2f]  最多 %d 个柱子",
    map_x_min_m_, map_x_max_m_, map_y_min_m_, map_y_max_m_, max_pillars_);
}

// ─────────────────────────────────────────────────────────────────────────────
// /pillar_detect_enable 回调：边沿触发
//   false→true：清空累积，开始收
//   true→false：聚类并发布结果
// ─────────────────────────────────────────────────────────────────────────────
void PillarDetectorTFNode::enableCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  const bool desired = msg->data;
  if (desired == enabled_) { return; }

  if (desired) {
    accumulated_.clear();
    frames_accumulated_ = 0;
    result_published_   = false;
    enabled_ = true;
    RCLCPP_INFO(get_logger(), "检测使能：开始累积...");
  } else {
    enabled_ = false;
    RCLCPP_INFO(get_logger(),
      "检测禁用：累积 %d 帧 / %zu 个候选点，开始聚类...",
      frames_accumulated_, accumulated_.size());
    if (!result_published_) {
      publishPillars(clusterDetections(accumulated_));
      result_published_ = true;
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// 单帧：先用 TF 把每个点变到 map 系，再用世界 bbox 过滤、按扫描序分组
// ─────────────────────────────────────────────────────────────────────────────
std::vector<DetectionTF> PillarDetectorTFNode::detectInFrame(
  const sensor_msgs::msg::LaserScan & scan)
{
  std::vector<DetectionTF> detections;
  const int n = static_cast<int>(scan.ranges.size());
  if (n < 8) { return detections; }

  // 查询本帧时间戳的 TF；失败则用最新
  geometry_msgs::msg::TransformStamped tf_msg;
  try {
    tf_msg = tf_buffer_->lookupTransform(
      map_frame_, laser_link_frame_, scan.header.stamp,
      rclcpp::Duration::from_seconds(tf_timeout_sec_));
  } catch (const tf2::TransformException &) {
    try {
      tf_msg = tf_buffer_->lookupTransform(map_frame_, laser_link_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "TF %s <- %s 查询失败: %s", map_frame_.c_str(), laser_link_frame_.c_str(), ex.what());
      return detections;
    }
  }

  tf2::Transform T_ml;
  tf2::fromMsg(tf_msg.transform, T_ml);

  struct Pt { double x, y; };
  std::vector<Pt> in_bounds;
  in_bounds.reserve(static_cast<std::size_t>(n) / 4);

  const double ang_min = static_cast<double>(scan.angle_min);
  const double ang_res = static_cast<double>(scan.angle_increment);

  for (int i = 0; i < n; ++i) {
    const float r = scan.ranges[i];
    if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max) { continue; }

    const double theta = ang_min + static_cast<double>(i) * ang_res;
    const double xl = static_cast<double>(r) * std::cos(theta);
    const double yl = static_cast<double>(r) * std::sin(theta);

    // laser_link 系 → map 系
    tf2::Vector3 p_map = T_ml * tf2::Vector3(xl, yl, 0.0);
    const double xm = p_map.x();
    const double ym = p_map.y();

    if (xm >= map_x_min_m_ && xm <= map_x_max_m_ &&
        ym >= map_y_min_m_ && ym <= map_y_max_m_)
    {
      in_bounds.push_back({xm, ym});
    }
  }

  if (in_bounds.empty()) { return detections; }

  // 按扫描角度序分组（相邻激光点在 map 系里也相邻）
  std::vector<std::vector<Pt>> groups;
  std::vector<Pt> cur_group = {in_bounds[0]};
  for (std::size_t k = 1; k < in_bounds.size(); ++k) {
    const Pt & prev = cur_group.back();
    const Pt & curr = in_bounds[k];
    const double dx = curr.x - prev.x;
    const double dy = curr.y - prev.y;
    if (std::sqrt(dx * dx + dy * dy) <= group_dist_m_) {
      cur_group.push_back(curr);
    } else {
      if (static_cast<int>(cur_group.size()) >= min_pts_per_group_) {
        groups.push_back(cur_group);
      }
      cur_group = {curr};
    }
  }
  if (static_cast<int>(cur_group.size()) >= min_pts_per_group_) {
    groups.push_back(cur_group);
  }

  for (const auto & g : groups) {
    double sx = 0.0, sy = 0.0;
    for (const auto & p : g) { sx += p.x; sy += p.y; }
    const double cx = sx / static_cast<double>(g.size());
    const double cy = sy / static_cast<double>(g.size());

    bool too_close = false;
    for (const auto & d : detections) {
      const double dx = cx - d.x_m;
      const double dy = cy - d.y_m;
      if (std::sqrt(dx * dx + dy * dy) < min_pillar_separation_m_) {
        too_close = true;
        break;
      }
    }
    if (!too_close) {
      detections.push_back({cx, cy});
    }
  }

  return detections;
}

// ─────────────────────────────────────────────────────────────────────────────
// 多帧累积点的连通聚类，按票数降序取前 max_pillars_ 个
// ─────────────────────────────────────────────────────────────────────────────
std::vector<ClusterTF> PillarDetectorTFNode::clusterDetections(
  const std::vector<DetectionTF> & dets) const
{
  const int nd = static_cast<int>(dets.size());
  std::vector<int> labels(nd, -1);
  int next_label = 0;

  for (int i = 0; i < nd; ++i) {
    if (labels[i] >= 0) { continue; }
    labels[i] = next_label;
    bool changed = true;
    while (changed) {
      changed = false;
      for (int j = 0; j < nd; ++j) {
        if (labels[j] >= 0) { continue; }
        for (int k = 0; k < nd; ++k) {
          if (labels[k] != next_label) { continue; }
          const double dx = dets[j].x_m - dets[k].x_m;
          const double dy = dets[j].y_m - dets[k].y_m;
          if (std::sqrt(dx * dx + dy * dy) < cluster_merge_dist_m_) {
            labels[j] = next_label;
            changed = true;
            break;
          }
        }
      }
    }
    ++next_label;
  }

  std::vector<ClusterTF> clusters;
  for (int lbl = 0; lbl < next_label; ++lbl) {
    double sx = 0.0, sy = 0.0;
    int count = 0;
    for (int i = 0; i < nd; ++i) {
      if (labels[i] == lbl) { sx += dets[i].x_m; sy += dets[i].y_m; ++count; }
    }
    if (count >= min_votes_) {
      clusters.push_back({sx / count, sy / count, count});
    }
  }

  std::sort(clusters.begin(), clusters.end(),
    [](const ClusterTF & a, const ClusterTF & b) { return a.votes > b.votes; });
  if (static_cast<int>(clusters.size()) > max_pillars_) {
    clusters.resize(max_pillars_);
  }
  return clusters;
}

void PillarDetectorTFNode::publishPillars(const std::vector<ClusterTF> & pillars)
{
  std_msgs::msg::Float32MultiArray msg;
  msg.data.resize(pillars.size() * 2);
  for (std::size_t k = 0; k < pillars.size(); ++k) {
    msg.data[k * 2]     = static_cast<float>(pillars[k].x_m);
    msg.data[k * 2 + 1] = static_cast<float>(pillars[k].y_m);
  }
  pillars_pub_->publish(msg);

  RCLCPP_INFO(get_logger(), " ");
  RCLCPP_INFO(get_logger(), "╔══════════════════════════════════════════╗");
  RCLCPP_INFO(get_logger(), "║   柱子检测结果 (map 系, 共 %zu 个)         ║", pillars.size());
  RCLCPP_INFO(get_logger(), "╠══════════════════════════════════════════╣");
  for (std::size_t k = 0; k < pillars.size(); ++k) {
    RCLCPP_INFO(get_logger(), "║  第 %zu 个柱子:", k + 1);
    RCLCPP_INFO(get_logger(), "║    x = %+.3f m   y = %+.3f m",
      pillars[k].x_m, pillars[k].y_m);
    RCLCPP_INFO(get_logger(), "║    票数: %d", pillars[k].votes);
  }
  RCLCPP_INFO(get_logger(), "║  已发布到 /detected_pillars");
  RCLCPP_INFO(get_logger(), "╚══════════════════════════════════════════╝");
  RCLCPP_INFO(get_logger(), " ");
}

void PillarDetectorTFNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!enabled_) { return; }

  const auto frame_dets = detectInFrame(*msg);
  ++frames_accumulated_;
  for (const auto & d : frame_dets) { accumulated_.push_back(d); }

  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
    "累积中: 帧=%d 本帧候选=%zu 累积点=%zu",
    frames_accumulated_, frame_dets.size(), accumulated_.size());
}

}  // namespace pillar_detector_pkg
