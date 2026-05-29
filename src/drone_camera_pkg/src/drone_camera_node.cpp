#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

class DroneCameraNode : public rclcpp::Node
{
public:
  DroneCameraNode()
  : Node("drone_camera_node"),
    camera_device_(declare_parameter<std::string>("camera_device", "/dev/video0")),
    frame_width_(declare_parameter<int>("frame_width", 640)),
    frame_height_(declare_parameter<int>("frame_height", 480)),
    fps_(declare_parameter<double>("fps", 15.0)),
    window_name_(declare_parameter<std::string>("window_name", "drone_camera_preview")),
    spray_allowed_topic_(declare_parameter<std::string>("spray_allowed_topic", "/spray_allowed")),
    fourcc_(declare_parameter<std::string>("fourcc", "MJPG")),
    rotate_code_(declare_parameter<int>("rotate_code", -1)),
    center_roi_width_(declare_parameter<int>("center_roi_width", 50)),
    center_roi_height_(declare_parameter<int>("center_roi_height", 50)),
    green_h_min_(declare_parameter<int>("green_h_min", 25)),
    green_h_max_(declare_parameter<int>("green_h_max", 100)),
    green_s_min_(declare_parameter<int>("green_s_min", 20)),
    green_v_min_(declare_parameter<int>("green_v_min", 40)),
    green_pixel_threshold_(declare_parameter<int>("green_pixel_threshold", 100))
  {
    spray_allowed_pub_ =
      create_publisher<std_msgs::msg::Bool>(spray_allowed_topic_, rclcpp::QoS(10));

    // 下视相机(down_cam，/dev/video0)。必须用 V4L2 后端 + 强制 MJPG，
    // 否则默认 YUYV 在高分辨率下只有 3fps，画面巨卡。
    if (!camera_.open(camera_device_, cv::CAP_V4L2)) {
      throw std::runtime_error("[down_cam] 打开下视相机失败: " + camera_device_);
    }

    // FOURCC 必须在设分辨率之前设置
    if (fourcc_.size() == 4) {
      camera_.set(
        cv::CAP_PROP_FOURCC,
        cv::VideoWriter::fourcc(fourcc_[0], fourcc_[1], fourcc_[2], fourcc_[3]));
    }
    if (frame_width_ > 0) {
      camera_.set(cv::CAP_PROP_FRAME_WIDTH, frame_width_);
    }
    if (frame_height_ > 0) {
      camera_.set(cv::CAP_PROP_FRAME_HEIGHT, frame_height_);
    }
    if (fps_ > 0.0) {
      camera_.set(cv::CAP_PROP_FPS, fps_);
    }

    // 打印实际生效的格式（驱动可能把请求值改掉）
    const int got_fourcc = static_cast<int>(camera_.get(cv::CAP_PROP_FOURCC));
    char fcc[5] = {
      static_cast<char>(got_fourcc & 0xFF),
      static_cast<char>((got_fourcc >> 8) & 0xFF),
      static_cast<char>((got_fourcc >> 16) & 0xFF),
      static_cast<char>((got_fourcc >> 24) & 0xFF),
      '\0'};
    RCLCPP_INFO(
      get_logger(),
      "[down_cam] 已打开 %s  fourcc=%s  %.0fx%.0f @ %.0ffps",
      camera_device_.c_str(), fcc,
      camera_.get(cv::CAP_PROP_FRAME_WIDTH),
      camera_.get(cv::CAP_PROP_FRAME_HEIGHT),
      camera_.get(cv::CAP_PROP_FPS));

    const auto period = std::chrono::duration<double>(1.0 / std::max(fps_, 1.0));
    frame_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&DroneCameraNode::frameTimerCallback, this));

    RCLCPP_INFO(
      get_logger(),
      "Camera color detector ready. camera=%s roi=%dx%d spray_topic=%s green_pixel_threshold=%d",
      camera_device_.c_str(),
      center_roi_width_,
      center_roi_height_,
      spray_allowed_topic_.c_str(),
      green_pixel_threshold_);
  }

  ~DroneCameraNode() override
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (camera_.isOpened()) {
      camera_.release();
    }
    cv::destroyAllWindows();
  }

private:
  void detectFieldColorAndPublish(const cv::Mat & frame)
  {
    const int roi_width = std::clamp(center_roi_width_, 1, frame.cols);
    const int roi_height = std::clamp(center_roi_height_, 1, frame.rows);
    const int roi_x = std::max(0, (frame.cols - roi_width) / 2);
    const int roi_y = std::max(0, (frame.rows - roi_height) / 2);
    const cv::Mat roi = frame(cv::Rect(roi_x, roi_y, roi_width, roi_height));

    cv::Mat hsv;
    cv::cvtColor(roi, hsv, cv::COLOR_BGR2HSV);

    cv::Mat green_mask;
    cv::inRange(
      hsv,
      cv::Scalar(green_h_min_, green_s_min_, green_v_min_),
      cv::Scalar(green_h_max_, 255, 255),
      green_mask);

    const int green_pixels = cv::countNonZero(green_mask);
    const int pixel_count = roi_width * roi_height;
    const double green_ratio = pixel_count > 0 ?
      static_cast<double>(green_pixels) / static_cast<double>(pixel_count) : 0.0;
    const bool allowed = green_pixels >= green_pixel_threshold_;

    std_msgs::msg::Bool allowed_msg;
    allowed_msg.data = allowed;
    spray_allowed_pub_->publish(allowed_msg);

    RCLCPP_DEBUG_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "Center color: green_pixels=%d/%d green_ratio=%.3f threshold_pixels=%d allowed=%s",
      green_pixels,
      pixel_count,
      green_ratio,
      green_pixel_threshold_,
      allowed ? "true" : "false");
  }

  void frameTimerCallback()
  {
    cv::Mat frame;
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      if (!camera_.isOpened()) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 3000, "Camera is not opened.");
        return;
      }

      if (!camera_.read(frame) || frame.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Failed to read frame from camera.");
        return;
      }
    }

    // rotate_code: -1=不转, 0=顺时针90, 1=180, 2=逆时针90（= cv::ROTATE_* 取值）
    if (rotate_code_ >= 0) {
      cv::rotate(frame, frame, rotate_code_);
    }

    detectFieldColorAndPublish(frame);
    cv::imshow(window_name_, frame);
    cv::waitKey(1);
  }

  std::string camera_device_;
  int frame_width_;
  int frame_height_;
  double fps_;
  std::string window_name_;
  std::string spray_allowed_topic_;
  std::string fourcc_;
  int rotate_code_;
  int center_roi_width_;
  int center_roi_height_;
  int green_h_min_;
  int green_h_max_;
  int green_s_min_;
  int green_v_min_;
  int green_pixel_threshold_;

  std::mutex frame_mutex_;
  cv::VideoCapture camera_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr spray_allowed_pub_;
  rclcpp::TimerBase::SharedPtr frame_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DroneCameraNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
