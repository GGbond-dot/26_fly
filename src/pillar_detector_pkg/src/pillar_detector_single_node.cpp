#include "pillar_detector_pkg/pillar_detector_single_node.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace pillar_detector_pkg
{

PillarDetectorSingleNode::PillarDetectorSingleNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("pillar_detector_single", options),
  frame_count_(0),
  done_(false),
  ranges_precomputed_(false)
{
  map_x_min_m_ = declare_parameter("map_x_min_m", 0.5);
  map_x_max_m_ = declare_parameter("map_x_max_m", 2.5);
  map_y_min_m_ = declare_parameter("map_y_min_m", -2.5);
  map_y_max_m_ = declare_parameter("map_y_max_m", -0.5);

  group_dist_m_ = declare_parameter("group_dist_m", 0.25);
  min_pts_per_group_ = declare_parameter("min_pts_per_group", 4);
  min_pillar_separation_m_ = declare_parameter("min_pillar_separation_m", 0.40);

  accumulation_frames_ = declare_parameter("accumulation_frames", 20);
  cluster_merge_dist_m_ = declare_parameter("cluster_merge_dist_m", 0.20);
  min_votes_ = declare_parameter("min_votes", 8);

  const std::string scan_topic = declare_parameter("scan_topic", std::string("/scan"));
  const std::string output_topic =
    declare_parameter("output_topic", std::string("/detected_pillar"));

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&PillarDetectorSingleNode::scanCallback, this, std::placeholders::_1));

  pillar_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
    output_topic,
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  RCLCPP_INFO(
    get_logger(),
    "Single pillar detector started: scan=%s output=%s frames=%d area x=[%.2f, %.2f] y=[%.2f, %.2f]",
    scan_topic.c_str(),
    output_topic.c_str(),
    accumulation_frames_,
    map_x_min_m_,
    map_x_max_m_,
    map_y_min_m_,
    map_y_max_m_);
}

void PillarDetectorSingleNode::precomputeMaxRanges(const sensor_msgs::msg::LaserScan & scan)
{
  const int count = static_cast<int>(scan.ranges.size());
  max_range_per_idx_.assign(count, std::numeric_limits<double>::infinity());

  for (int i = 0; i < count; ++i) {
    const double theta =
      static_cast<double>(scan.angle_min) +
      static_cast<double>(i) * static_cast<double>(scan.angle_increment);
    const double cos_t = std::cos(theta);
    const double sin_t = std::sin(theta);
    double max_range = std::numeric_limits<double>::infinity();

    if (cos_t > 1e-9) {
      max_range = std::min(max_range, map_x_max_m_ / cos_t);
    }
    if (sin_t < -1e-9) {
      max_range = std::min(max_range, (-map_y_min_m_) / (-sin_t));
    }

    max_range_per_idx_[i] = max_range;
  }

  ranges_precomputed_ = true;
}

std::vector<SingleDetection> PillarDetectorSingleNode::detectInFrame(
  const sensor_msgs::msg::LaserScan & scan)
{
  std::vector<SingleDetection> detections;
  const int count = static_cast<int>(scan.ranges.size());
  if (count < 8) {
    return detections;
  }

  const int seg_start = count / 4;
  const int seg_end = count / 2;
  const double angle_increment = static_cast<double>(scan.angle_increment);

  struct Point
  {
    double x;
    double y;
  };
  std::vector<Point> in_bounds;

  for (int i = seg_start; i < seg_end; ++i) {
    const float range = scan.ranges[i];
    if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) {
      continue;
    }
    if (static_cast<double>(range) > max_range_per_idx_[i]) {
      continue;
    }

    const double theta =
      static_cast<double>(scan.angle_min) + static_cast<double>(i) * angle_increment;
    const double x = static_cast<double>(range) * std::cos(theta);
    const double y = static_cast<double>(range) * std::sin(theta);

    if (x >= map_x_min_m_ && x <= map_x_max_m_ && y >= map_y_min_m_ && y <= map_y_max_m_) {
      in_bounds.push_back({x, y});
    }
  }

  if (in_bounds.empty()) {
    return detections;
  }

  std::vector<std::vector<Point>> groups;
  std::vector<Point> current_group{in_bounds.front()};

  for (std::size_t i = 1; i < in_bounds.size(); ++i) {
    const auto & prev = current_group.back();
    const auto & curr = in_bounds[i];
    const double dist = std::hypot(curr.x - prev.x, curr.y - prev.y);
    if (dist <= group_dist_m_) {
      current_group.push_back(curr);
      continue;
    }

    if (static_cast<int>(current_group.size()) >= min_pts_per_group_) {
      groups.push_back(current_group);
    }
    current_group = {curr};
  }

  if (static_cast<int>(current_group.size()) >= min_pts_per_group_) {
    groups.push_back(current_group);
  }

  for (const auto & group : groups) {
    double sum_x = 0.0;
    double sum_y = 0.0;
    for (const auto & point : group) {
      sum_x += point.x;
      sum_y += point.y;
    }

    const double x = sum_x / static_cast<double>(group.size());
    const double y = sum_y / static_cast<double>(group.size());

    bool too_close = false;
    for (const auto & detection : detections) {
      if (std::hypot(x - detection.x_m, y - detection.y_m) < min_pillar_separation_m_) {
        too_close = true;
        break;
      }
    }
    if (!too_close) {
      detections.push_back({x, y});
    }
  }

  return detections;
}

bool PillarDetectorSingleNode::findBestPillar(
  const std::vector<SingleDetection> & detections,
  SingleCluster & pillar) const
{
  const int count = static_cast<int>(detections.size());
  if (count == 0) {
    return false;
  }

  std::vector<int> labels(count, -1);
  int next_label = 0;

  for (int i = 0; i < count; ++i) {
    if (labels[i] >= 0) {
      continue;
    }

    labels[i] = next_label;
    bool changed = true;
    while (changed) {
      changed = false;
      for (int j = 0; j < count; ++j) {
        if (labels[j] >= 0) {
          continue;
        }
        for (int k = 0; k < count; ++k) {
          if (labels[k] != next_label) {
            continue;
          }
          if (
            std::hypot(
              detections[j].x_m - detections[k].x_m,
              detections[j].y_m - detections[k].y_m) < cluster_merge_dist_m_)
          {
            labels[j] = next_label;
            changed = true;
            break;
          }
        }
      }
    }
    ++next_label;
  }

  bool found = false;
  SingleCluster best{0.0, 0.0, 0};
  for (int label = 0; label < next_label; ++label) {
    double sum_x = 0.0;
    double sum_y = 0.0;
    int votes = 0;
    for (int i = 0; i < count; ++i) {
      if (labels[i] == label) {
        sum_x += detections[i].x_m;
        sum_y += detections[i].y_m;
        ++votes;
      }
    }

    if (votes >= min_votes_ && votes > best.votes) {
      best = SingleCluster{sum_x / static_cast<double>(votes), sum_y / static_cast<double>(votes), votes};
      found = true;
    }
  }

  if (!found) {
    return false;
  }

  pillar = best;
  return true;
}

void PillarDetectorSingleNode::publishPillar(const SingleCluster & pillar)
{
  std_msgs::msg::Float32MultiArray msg;
  msg.data = {
    static_cast<float>(pillar.x_m),
    static_cast<float>(pillar.y_m)};
  pillar_pub_->publish(msg);

  RCLCPP_INFO(
    get_logger(),
    "Detected pillar: x=%.3fm y=%.3fm votes=%d/%d. Published [x, y].",
    pillar.x_m,
    pillar.y_m,
    pillar.votes,
    accumulation_frames_);
}

void PillarDetectorSingleNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (done_) {
    return;
  }

  if (!ranges_precomputed_) {
    precomputeMaxRanges(*msg);
  }

  const auto detections = detectInFrame(*msg);
  ++frame_count_;
  accumulated_.insert(accumulated_.end(), detections.begin(), detections.end());

  RCLCPP_INFO_THROTTLE(
    get_logger(),
    *get_clock(),
    1000,
    "Pillar detection frame %d/%d: frame_candidates=%zu accumulated=%zu",
    frame_count_,
    accumulation_frames_,
    detections.size(),
    accumulated_.size());

  if (frame_count_ < accumulation_frames_) {
    return;
  }

  done_ = true;
  SingleCluster pillar;
  if (findBestPillar(accumulated_, pillar)) {
    publishPillar(pillar);
  } else {
    RCLCPP_WARN(
      get_logger(),
      "No pillar reached min_votes=%d after %d frames.",
      min_votes_,
      accumulation_frames_);
  }
}

}  // namespace pillar_detector_pkg
