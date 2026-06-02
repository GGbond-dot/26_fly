// standoff_estimator_node
//
// 用 2D 激光雷达 /scan 测「飞机到货架板面」的垂直距离(standoff)，给视觉做
// 像素误差 → 真实 cm 的尺度换算用。
//
// 思路：货架两面都是实板(不镂空)，所以在相机正前方一个扇区里，激光点会落成
// 一条直线(板面在水平面的截线)。对扇区内的点做 PCA 直线拟合：
//   - 原点到该直线的垂距 = standoff 距离(米)
//   - 直线法向与「正前方」的夹角 = 板面是否正对(yaw 是否摆正)
// 纯在 laser_link 系里算，不依赖 tf / cartographer，雷达一上来就能用。
//
// 发布：
//   <distance_topic> std_msgs/Float32  standoff 垂距(米)
//   <angle_topic>    std_msgs/Float32  板面法向相对正前方夹角(弧度，0=正对)
//   <valid_topic>    std_msgs/Bool     本帧是否拟合成功

#include <algorithm>
#include <cmath>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

namespace
{
// 把角度归一化到 [-pi, pi]
double wrapPi(double a)
{
  while (a > M_PI) { a -= 2.0 * M_PI; }
  while (a < -M_PI) { a += 2.0 * M_PI; }
  return a;
}
}  // namespace

class StandoffEstimator : public rclcpp::Node
{
public:
  StandoffEstimator()
  : rclcpp::Node("standoff_estimator")
  {
    scan_topic_     = declare_parameter<std::string>("scan_topic", "/scan");
    distance_topic_ = declare_parameter<std::string>("distance_topic", "/standoff/distance");
    angle_topic_    = declare_parameter<std::string>("angle_topic", "/standoff/board_angle");
    valid_topic_    = declare_parameter<std::string>("valid_topic", "/standoff/valid");

    // 正前方方向(雷达 x 轴为 0)。若雷达 x 轴和相机朝向有安装偏差，用它对齐。
    sector_center_deg_     = declare_parameter<double>("sector_center_deg", 0.0);
    // 取正前方 ±half 的扇区点参与拟合
    sector_half_width_deg_ = declare_parameter<double>("sector_half_width_deg", 30.0);

    min_points_      = declare_parameter<int>("min_points", 10);
    min_range_m_     = declare_parameter<double>("min_range_m", 0.05);
    max_range_m_     = declare_parameter<double>("max_range_m", 8.0);
    // 首次拟合后，剔除离直线超过该阈值的离群点再拟合一次
    outlier_thresh_m_ = declare_parameter<double>("outlier_thresh_m", 0.08);
    // 距离输出的 EMA 平滑系数(0~1，越大越平滑)
    ema_alpha_       = declare_parameter<double>("ema_alpha", 0.5);

    sector_center_rad_ = sector_center_deg_ * M_PI / 180.0;
    sector_half_rad_   = sector_half_width_deg_ * M_PI / 180.0;

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&StandoffEstimator::scanCallback, this, std::placeholders::_1));

    dist_pub_  = create_publisher<std_msgs::msg::Float32>(distance_topic_, 10);
    angle_pub_ = create_publisher<std_msgs::msg::Float32>(angle_topic_, 10);
    valid_pub_ = create_publisher<std_msgs::msg::Bool>(valid_topic_, 10);

    RCLCPP_INFO(get_logger(),
      "standoff_estimator 启动: scan='%s' 扇区中心=%.1f° 半宽=±%.1f° -> dist='%s' angle='%s'",
      scan_topic_.c_str(), sector_center_deg_, sector_half_width_deg_,
      distance_topic_.c_str(), angle_topic_.c_str());
  }

private:
  struct Pt { double x, y; };

  // 对一组 (x,y) 点做 PCA 直线拟合，返回原点到直线的垂距与直线法向角。
  // 成功返回 true。
  bool fitLine(const std::vector<Pt> & pts, double & perp_dist, double & normal_angle) const
  {
    const std::size_t n = pts.size();
    if (n < 2) { return false; }

    double cx = 0.0, cy = 0.0;
    for (const auto & p : pts) { cx += p.x; cy += p.y; }
    cx /= static_cast<double>(n);
    cy /= static_cast<double>(n);

    double sxx = 0.0, sxy = 0.0, syy = 0.0;
    for (const auto & p : pts) {
      const double dx = p.x - cx;
      const double dy = p.y - cy;
      sxx += dx * dx;
      sxy += dx * dy;
      syy += dy * dy;
    }

    // 主轴(方差最大方向)= 直线方向；法向 = 主轴 + 90°
    const double theta_line = 0.5 * std::atan2(2.0 * sxy, sxx - syy);
    normal_angle = theta_line + M_PI / 2.0;

    const double nx = std::cos(normal_angle);
    const double ny = std::sin(normal_angle);
    perp_dist = std::fabs(cx * nx + cy * ny);  // 原点到直线垂距
    return true;
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    const int nbeam = static_cast<int>(msg->ranges.size());
    if (nbeam < 2) { publishInvalid(); return; }

    const double lo = std::max(static_cast<double>(msg->range_min), min_range_m_);
    const double hi = std::min(static_cast<double>(msg->range_max), max_range_m_);

    std::vector<Pt> pts;
    pts.reserve(static_cast<std::size_t>(nbeam) / 4);

    for (int i = 0; i < nbeam; ++i) {
      const float r = msg->ranges[i];
      if (!std::isfinite(r) || r < lo || r > hi) { continue; }

      const double theta = static_cast<double>(msg->angle_min) +
                           static_cast<double>(i) * static_cast<double>(msg->angle_increment);
      // 只要正前方扇区内的点
      if (std::fabs(wrapPi(theta - sector_center_rad_)) > sector_half_rad_) { continue; }

      pts.push_back({static_cast<double>(r) * std::cos(theta),
                     static_cast<double>(r) * std::sin(theta)});
    }

    if (static_cast<int>(pts.size()) < min_points_) { publishInvalid(); return; }

    double perp_dist = 0.0, normal_angle = 0.0;
    if (!fitLine(pts, perp_dist, normal_angle)) { publishInvalid(); return; }

    // 一遍离群点剔除后重拟合，压掉边缘/缝隙的杂点
    {
      const double nx = std::cos(normal_angle);
      const double ny = std::sin(normal_angle);
      std::vector<Pt> inliers;
      inliers.reserve(pts.size());
      for (const auto & p : pts) {
        if (std::fabs(p.x * nx + p.y * ny - perp_dist) <= outlier_thresh_m_) {
          inliers.push_back(p);
        }
      }
      if (static_cast<int>(inliers.size()) >= min_points_) {
        fitLine(inliers, perp_dist, normal_angle);
      }
    }

    // 法向相对正前方的夹角，折到 [-pi/2, pi/2]，正对时 ≈ 0
    double board_angle = wrapPi(normal_angle - sector_center_rad_);
    if (board_angle > M_PI / 2.0) { board_angle -= M_PI; }
    if (board_angle < -M_PI / 2.0) { board_angle += M_PI; }

    // 距离 EMA 平滑
    if (!has_filt_) { dist_filt_ = perp_dist; has_filt_ = true; }
    else { dist_filt_ = ema_alpha_ * dist_filt_ + (1.0 - ema_alpha_) * perp_dist; }

    std_msgs::msg::Float32 dmsg; dmsg.data = static_cast<float>(dist_filt_);
    std_msgs::msg::Float32 amsg; amsg.data = static_cast<float>(board_angle);
    std_msgs::msg::Bool vmsg; vmsg.data = true;
    dist_pub_->publish(dmsg);
    angle_pub_->publish(amsg);
    valid_pub_->publish(vmsg);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "standoff=%.3f m (raw %.3f, %zu点)  板面角=%.1f°",
      dist_filt_, perp_dist, pts.size(), board_angle * 180.0 / M_PI);
  }

  void publishInvalid()
  {
    has_filt_ = false;
    std_msgs::msg::Bool vmsg; vmsg.data = false;
    valid_pub_->publish(vmsg);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
      "扇区内有效点不足，无法拟合板面距离");
  }

  // 参数
  std::string scan_topic_, distance_topic_, angle_topic_, valid_topic_;
  double sector_center_deg_, sector_half_width_deg_;
  double sector_center_rad_, sector_half_rad_;
  int    min_points_;
  double min_range_m_, max_range_m_, outlier_thresh_m_, ema_alpha_;

  // 状态
  bool   has_filt_ {false};
  double dist_filt_ {0.0};

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dist_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angle_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr valid_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StandoffEstimator>());
  rclcpp::shutdown();
  return 0;
}
