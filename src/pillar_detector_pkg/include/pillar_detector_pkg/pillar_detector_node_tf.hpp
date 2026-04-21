#pragma once

#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace pillar_detector_pkg
{

struct DetectionTF
{
  double x_m;
  double y_m;
};

struct ClusterTF
{
  double x_m;
  double y_m;
  int    votes;
};

// 飞行中柱子检测：靠 TF(map <- laser_link) 把每帧激光点变换到 map 系，
// 再用世界 bbox 过滤、分组、多帧聚类。由 /pillar_detect_enable 控制启停。
class PillarDetectorTFNode : public rclcpp::Node
{
public:
  explicit PillarDetectorTFNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void enableCallback(const std_msgs::msg::Bool::SharedPtr msg);

  std::vector<DetectionTF> detectInFrame(const sensor_msgs::msg::LaserScan & scan);
  std::vector<ClusterTF>   clusterDetections(const std::vector<DetectionTF> & dets) const;
  void publishPillars(const std::vector<ClusterTF> & pillars);

  // ── 参数 ──────────────────────────────────────────────────
  std::string scan_topic_;
  std::string enable_topic_;
  std::string map_frame_;
  std::string laser_link_frame_;

  // 柱子有效世界区域（map 系，单位米）
  double map_x_min_m_;
  double map_x_max_m_;
  double map_y_min_m_;
  double map_y_max_m_;

  // 单帧分组
  int    min_pts_per_group_;
  double group_dist_m_;
  double min_pillar_separation_m_;

  // 多帧聚类
  double cluster_merge_dist_m_;
  int    min_votes_;
  int    max_pillars_;

  double tf_timeout_sec_;

  // ── 状态 ──────────────────────────────────────────────────
  bool enabled_;
  bool result_published_;
  int  frames_accumulated_;
  std::vector<DetectionTF> accumulated_;
  mutable std::mutex mutex_;

  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr         enable_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pillars_pub_;
};

}  // namespace pillar_detector_pkg
