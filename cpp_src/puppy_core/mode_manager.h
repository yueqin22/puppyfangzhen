// Mode Manager: robot mode state machine.
//
// 管理机器人运行模式，按优先级切换 (gaijin2.md 第6.2节)。
// FAULT 和 SAFE_STOP 优先级最高，可中断任何其他模式。
//
// Modes:
//   IDLE, READY, TELEOP, NAVIGATION, PATROL, SECURITY, DOCKING, SAFE_STOP, FAULT
#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "std_msgs/msg/string.hpp"
#include "puppy_interfaces/msg/robot_mode.hpp"
#include "puppy_interfaces/msg/battery_status.hpp"
#include "puppy_interfaces/msg/fall_event.hpp"

namespace puppy_core {

/// Manages robot operational modes with safety constraints.
class ModeManagerNode : public rclcpp::Node {
 public:
  ModeManagerNode();
  ~ModeManagerNode() override = default;

 private:
  // Handle battery status - trigger SAFE_STOP on critical battery.
  void onBattery(const puppy_interfaces::msg::BatteryStatus::SharedPtr msg);
  // Handle fall event - immediately enter SAFE_STOP.
  void onFall(const puppy_interfaces::msg::FallEvent::SharedPtr msg);
  // Handle external mode request.
  void onModeRequest(const std_msgs::msg::String::SharedPtr msg);

  // Switch mode with priority checking.
  void switchMode(const std::string& new_mode, const std::string& reason = "");

  // Broadcast current mode.
  void publishMode();

 private:
  std::string current_mode_{"IDLE"};
  std::string previous_mode_{"IDLE"};
  bool motion_enabled_{false};
  bool autonomy_enabled_{false};
  bool mode_lock_{false};  // True when in FAULT/SAFE_STOP

  // Mode priority (higher number = higher priority)
  static const std::unordered_map<std::string, int> MODE_PRIORITY;
  static const std::unordered_set<std::string> VALID_MODES;

  rclcpp::Publisher<puppy_interfaces::msg::RobotMode>::SharedPtr mode_pub_;

  rclcpp::Subscription<puppy_interfaces::msg::BatteryStatus>::SharedPtr
      battery_sub_;
  rclcpp::Subscription<puppy_interfaces::msg::FallEvent>::SharedPtr fall_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_request_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace puppy_core
