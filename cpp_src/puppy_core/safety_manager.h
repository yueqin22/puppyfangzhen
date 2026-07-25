// Safety Manager: monitors safety conditions and triggers emergency response.
//
// 实现安全要求 (gaijin2.md 第14节):
//   1. Command timeout auto-stop
//   2. Mode switch clears motion commands
//   3. Fall detection stops navigation
//   4. Low battery prevents patrol
//   5. Driver error enters SAFE_STOP
//   6. Motors disabled by default on startup
#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "puppy_interfaces/msg/battery_status.hpp"
#include "puppy_interfaces/msg/fall_event.hpp"

namespace puppy_core {

/// Monitors safety conditions and enforces emergency responses.
class SafetyManagerNode : public rclcpp::Node {
 public:
  SafetyManagerNode();
  ~SafetyManagerNode() override = default;

 private:
  // Forward cmd_vel with safety checks.
  void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg);
  // Monitor battery for safety thresholds.
  void onBattery(const puppy_interfaces::msg::BatteryStatus::SharedPtr msg);
  // Fall detection - immediately disable motors.
  void onFall(const puppy_interfaces::msg::FallEvent::SharedPtr msg);
  // Handle motor enable/disable requests.
  void onMotorRequest(const std_msgs::msg::Bool::SharedPtr msg);

  // Periodic safety check - command timeout.
  void safetyCheck();

  void publishMotorEnable();

 private:
  // 安全参数（可由 safety.yaml 覆盖）
  double cmd_vel_timeout_{1.0};
  double low_battery_threshold_{0.20};
  double critical_battery_threshold_{0.10};
  double max_linear_x_{0.3};
  double max_angular_z_{1.2};
  // Default: disabled (gaijin2.md 14.6)
  bool motors_enabled_{false};

  rclcpp::Time last_cmd_vel_time_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr safe_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motor_enable_pub_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<puppy_interfaces::msg::BatteryStatus>::SharedPtr
      battery_sub_;
  rclcpp::Subscription<puppy_interfaces::msg::FallEvent>::SharedPtr fall_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr motor_request_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace puppy_core
