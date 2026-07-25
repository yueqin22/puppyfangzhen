// Safety Manager 实现
#include "puppy_core/safety_manager.h"

#include <algorithm>

namespace puppy_core {

SafetyManagerNode::SafetyManagerNode() : rclcpp::Node("safety_manager") {
  // P1-2: 安全参数从 declare_parameter 读取（可由 safety.yaml 覆盖）
  this->declare_parameter("cmd_vel_timeout", 1.0);
  this->declare_parameter("low_battery_threshold", 0.20);
  this->declare_parameter("critical_battery_threshold", 0.10);
  this->declare_parameter("max_linear_x", 0.3);
  this->declare_parameter("max_angular_z", 1.2);
  this->declare_parameter("motors_enabled_on_start", false);

  cmd_vel_timeout_ = this->get_parameter("cmd_vel_timeout").as_double();
  max_linear_x_ = this->get_parameter("max_linear_x").as_double();
  max_angular_z_ = this->get_parameter("max_angular_z").as_double();
  low_battery_threshold_ =
      this->get_parameter("low_battery_threshold").as_double();
  critical_battery_threshold_ =
      this->get_parameter("critical_battery_threshold").as_double();
  last_cmd_vel_time_ = this->now();
  // Default: disabled (gaijin2.md 14.6)
  motors_enabled_ = this->get_parameter("motors_enabled_on_start").as_bool();

  // Publishers
  safe_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel_safe", 10);
  motor_enable_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/robot/motion_enable", 10);

  // Subscribers
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&SafetyManagerNode::onCmdVel, this, std::placeholders::_1));
  battery_sub_ =
      this->create_subscription<puppy_interfaces::msg::BatteryStatus>(
          "/battery_status", 10,
          std::bind(&SafetyManagerNode::onBattery, this,
                    std::placeholders::_1));
  fall_sub_ = this->create_subscription<puppy_interfaces::msg::FallEvent>(
      "/fall/event", 10,
      std::bind(&SafetyManagerNode::onFall, this, std::placeholders::_1));
  motor_request_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/robot/motors_enable_request", 10,
      std::bind(&SafetyManagerNode::onMotorRequest, this,
                std::placeholders::_1));

  // Safety check timer (20Hz)
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&SafetyManagerNode::safetyCheck, this));

  RCLCPP_INFO(this->get_logger(),
              "Safety manager started (motors DISABLED by default)");
}

void SafetyManagerNode::onCmdVel(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  // Forward cmd_vel with safety checks.
  last_cmd_vel_time_ = this->now();
  if (!motors_enabled_) {
    // Motors disabled - send zero velocity
    geometry_msgs::msg::Twist zero;
    safe_cmd_pub_->publish(zero);
    return;
  }
  // Apply safety limits
  geometry_msgs::msg::Twist safe_msg;
  safe_msg.linear.x = std::max(-max_linear_x_,
                               std::min(max_linear_x_, msg->linear.x));
  safe_msg.angular.z = std::max(-max_angular_z_,
                                std::min(max_angular_z_, msg->angular.z));
  safe_cmd_pub_->publish(safe_msg);
}

void SafetyManagerNode::onBattery(
    const puppy_interfaces::msg::BatteryStatus::SharedPtr msg) {
  // Monitor battery for safety thresholds.
  if (msg->critical_battery) {
    RCLCPP_ERROR(this->get_logger(), "Critical battery! Entering SAFE_STOP");
    motors_enabled_ = false;
    publishMotorEnable();
  }
}

void SafetyManagerNode::onFall(
    const puppy_interfaces::msg::FallEvent::SharedPtr msg) {
  // Fall detection - immediately disable motors.
  if (msg->detected) {
    RCLCPP_ERROR(this->get_logger(),
                 "Fall detected (%s)! Stopping motors",
                 msg->direction.c_str());
    motors_enabled_ = false;
    publishMotorEnable();
  }
}

void SafetyManagerNode::onMotorRequest(
    const std_msgs::msg::Bool::SharedPtr msg) {
  // Handle motor enable/disable requests.
  motors_enabled_ = msg->data;
  RCLCPP_INFO(this->get_logger(), "Motors %s",
              msg->data ? "enabled" : "disabled");
  publishMotorEnable();
}

void SafetyManagerNode::safetyCheck() {
  // Periodic safety check - command timeout.
  if (motors_enabled_) {
    double elapsed = (this->now() - last_cmd_vel_time_).seconds();
    if (elapsed > cmd_vel_timeout_) {
      // Command timeout - publish zero velocity
      geometry_msgs::msg::Twist zero;
      safe_cmd_pub_->publish(zero);
    }
  }
}

void SafetyManagerNode::publishMotorEnable() {
  std_msgs::msg::Bool msg;
  msg.data = motors_enabled_;
  motor_enable_pub_->publish(msg);
}

}  // namespace puppy_core
