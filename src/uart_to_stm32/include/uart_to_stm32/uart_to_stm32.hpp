#ifndef UART_TO_STM32__UART_TO_STM32_HPP_
#define UART_TO_STM32__UART_TO_STM32_HPP_

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <serial_comm/serial_comm.h>

namespace uart_to_stm32
{

class UartToStm32
{
public:
  explicit UartToStm32(rclcpp::Node::SharedPtr node);
  ~UartToStm32();

  bool initialize(double update_rate, const std::string & source_frame, const std::string & target_frame);

private:
  void lookupTransform();
  void processTfTransform(const geometry_msgs::msg::TransformStamped & transform);
  void routeChoiceCallback(const std_msgs::msg::UInt8::SharedPtr msg);
  void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void targetVelocityCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
  Eigen::Vector3d transformVelocity(const Eigen::Vector3d & linear, double yaw);
  void sendVelocityToSerial(const Eigen::Vector3d & transformed_velocity);
  void sendTargetVelocityToSerial(float vx_cm_per_s, float vy_cm_per_s, float vz_cm_per_s, float vyaw_deg_per_s);
  void sendServoToSerial(uint8_t state);
  void sendElectromagnetToSerial(uint8_t state);
  void sendMissionCompleteToSerial();
  void servoControlCallback(const std_msgs::msg::UInt8::SharedPtr msg);
  void electromagnetControlCallback(const std_msgs::msg::UInt8::SharedPtr msg);
  void buzzerLedControlCallback(const std_msgs::msg::UInt8::SharedPtr msg);
  void sendBuzzerLedToSerial(uint8_t state);
  void ledDigitCallback(const std_msgs::msg::UInt8::SharedPtr msg);
  void sendLedDigitToSerial(uint8_t digit);
  void missionCompleteCallback(const std_msgs::msg::Empty::SharedPtr msg);
  void laserGroundHeightCallback(const std_msgs::msg::Int16::SharedPtr msg);
  void sendLaserGroundHeightToSerial(int16_t height_cm);
  void protocolDataHandler(uint8_t id, const std::vector<uint8_t> & data);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
  double update_rate_;
  std::string source_frame_;
  std::string target_frame_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr route_choice_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr target_velocity_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr servo_control_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr electromagnet_control_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr buzzer_led_control_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr led_digit_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr mission_complete_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr laser_ground_height_sub_;

  std::unique_ptr<serial_comm::SerialComm> serial_comm_;

  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr height_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr is_st_ready_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr mission_step_pub_;

  double current_yaw_;
  bool yaw_valid_;
  geometry_msgs::msg::Twist current_velocity_;
  bool velocity_valid_;
  bool route_task_active_;
  bool has_st_ready_pub_;

  static constexpr uint8_t VELOCITY_FRAME_ID = 0x32;
  static constexpr uint8_t TARGET_VELOCITY_FRAME_ID = 0x31;
  static constexpr uint8_t ST_READY_QUERY_ID = 0xF1;
  static constexpr uint8_t SERVO_FRAME_ID = 0x11;          // 0x01=放下机械臂, 0x00=收起机械臂
  static constexpr uint8_t ELECTROMAGNET_FRAME_ID = 0x33;  // 0x01=通电(吸), 0x00=断电(松)
  static constexpr uint8_t BUZZER_LED_FRAME_ID = 0x22;     // 0x01=蜂鸣器+LED开, 0x00=关（声光绑同一帧）
  // G 题（spray-task）新增：1B digit, LED 闪烁该次数显示条形码数字
  static constexpr uint8_t LED_DIGIT_FRAME_ID = 0x12;
  static constexpr uint8_t MISSION_COMPLETE_FRAME_ID = 0x66;
  static constexpr uint8_t MISSION_COMPLETE_VALUE = 0x06;
  static constexpr uint8_t LASER_GROUND_HEIGHT_FRAME_ID = 0x07;  // 2B int16 little-endian, 单位 cm
};

}  // 命名空间 uart_to_stm32

#endif  // 头文件保护宏 UART_TO_STM32__UART_TO_STM32_HPP_
