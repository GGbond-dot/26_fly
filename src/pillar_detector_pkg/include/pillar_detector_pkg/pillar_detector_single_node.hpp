#pragma once

#include <memory>
#include <mutex>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

namespace pillar_detector_pkg
{

// G 题（植保飞行器）杆塔检测：只输出**单一**杆塔坐标 (x, y)。
// 与原 PillarDetectorNode（多杆，输出 /detected_pillars）并存，互不影响。
// 朋友 26sunmmer_test2 的实现，结构名加 Single 区分。
struct SingleDetection
{
  double x_m;
  double y_m;
};

struct SingleCluster
{
  double x_m;
  double y_m;
  int votes;
};

class PillarDetectorSingleNode : public rclcpp::Node
{
public:
  explicit PillarDetectorSingleNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void precomputeMaxRanges(const sensor_msgs::msg::LaserScan & scan);
  std::vector<SingleDetection> detectInFrame(const sensor_msgs::msg::LaserScan & scan);
  bool findBestPillar(const std::vector<SingleDetection> & detections, SingleCluster & pillar) const;
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void publishPillar(const SingleCluster & pillar);

  double map_x_min_m_;
  double map_x_max_m_;
  double map_y_min_m_;
  double map_y_max_m_;

  int min_pts_per_group_;
  double group_dist_m_;
  double min_pillar_separation_m_;

  int accumulation_frames_;
  double cluster_merge_dist_m_;
  int min_votes_;

  int frame_count_;
  bool done_;
  bool ranges_precomputed_;
  std::vector<double> max_range_per_idx_;
  std::vector<SingleDetection> accumulated_;
  mutable std::mutex mutex_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pillar_pub_;
};

}  // namespace pillar_detector_pkg
