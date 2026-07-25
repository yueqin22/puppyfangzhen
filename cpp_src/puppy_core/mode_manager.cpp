// Mode Manager 实现
#include "puppy_core/mode_manager.h"

#include <algorithm>
#include <cctype>

namespace puppy_core {

// Mode priority (higher number = higher priority)
const std::unordered_map<std::string, int> ModeManagerNode::MODE_PRIORITY = {
    {"IDLE", 0},      {"READY", 1},      {"TELEOP", 2},
    {"NAVIGATION", 3},{"PATROL", 4},     {"SECURITY", 5},
    {"DOCKING", 6},   {"SAFE_STOP", 90}, {"FAULT", 100},
};

const std::unordered_set<std::string> ModeManagerNode::VALID_MODES = {
    "IDLE", "READY",   "TELEOP",    "NAVIGATION", "PATROL",
    "SECURITY", "DOCKING", "SAFE_STOP", "FAULT",
};

ModeManagerNode::ModeManagerNode() : rclcpp::Node("mode_manager") {
  // Publishers
  rclcpp::QoS qos(10);
  qos.reliable();
  mode_pub_ = this->create_publisher<puppy_interfaces::msg::RobotMode>(
      "/robot/mode", qos);

  // Subscribers
  battery_sub_ =
      this->create_subscription<puppy_interfaces::msg::BatteryStatus>(
          "/battery_status", 10,
          std::bind(&ModeManagerNode::onBattery, this,
                    std::placeholders::_1));
  fall_sub_ = this->create_subscription<puppy_interfaces::msg::FallEvent>(
      "/fall/event", 10,
      std::bind(&ModeManagerNode::onFall, this, std::placeholders::_1));
  mode_request_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/robot/mode_request", 10,
      std::bind(&ModeManagerNode::onModeRequest, this,
                std::placeholders::_1));

  // Mode broadcast timer (5Hz)
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&ModeManagerNode::publishMode, this));

  RCLCPP_INFO(this->get_logger(), "Mode manager started (mode=IDLE)");
}

void ModeManagerNode::onBattery(
    const puppy_interfaces::msg::BatteryStatus::SharedPtr msg) {
  // Handle battery status - trigger SAFE_STOP on critical battery.
  if (msg->critical_battery && current_mode_ != "FAULT") {
    switchMode("SAFE_STOP", "critical battery");
  } else if (msg->low_battery && current_mode_ == "PATROL") {
    // Interrupt patrol for docking
    switchMode("DOCKING", "low battery during patrol");
  }
}

void ModeManagerNode::onFall(
    const puppy_interfaces::msg::FallEvent::SharedPtr msg) {
  // Handle fall event - immediately enter SAFE_STOP.
  if (msg->detected && current_mode_ != "FAULT") {
    switchMode("SAFE_STOP", "fall detected: " + msg->direction);
  }
}

void ModeManagerNode::onModeRequest(
    const std_msgs::msg::String::SharedPtr msg) {
  // Handle external mode request.
  std::string requested = msg->data;
  // 转大写
  std::transform(requested.begin(), requested.end(), requested.begin(),
                 [](unsigned char c) { return std::toupper(c); });
  if (VALID_MODES.find(requested) == VALID_MODES.end()) {
    RCLCPP_WARN(this->get_logger(), "Invalid mode request: %s",
                requested.c_str());
    return;
  }
  switchMode(requested, "external request");
}

void ModeManagerNode::switchMode(const std::string& new_mode,
                                 const std::string& reason) {
  // Switch mode with priority checking.
  if (mode_lock_) {
    auto it_new = MODE_PRIORITY.find(new_mode);
    auto it_safe = MODE_PRIORITY.find("SAFE_STOP");
    if (it_new != MODE_PRIORITY.end() && it_safe != MODE_PRIORITY.end() &&
        it_new->second < it_safe->second) {
      RCLCPP_WARN(this->get_logger(),
                  "Mode switch to %s blocked (locked in %s)",
                  new_mode.c_str(), current_mode_.c_str());
      return;
    }
  }

  previous_mode_ = current_mode_;
  current_mode_ = new_mode;

  // Update enable flags
  motion_enabled_ = (new_mode != "IDLE" && new_mode != "SAFE_STOP" &&
                     new_mode != "FAULT");
  autonomy_enabled_ = (new_mode == "NAVIGATION" || new_mode == "PATROL" ||
                       new_mode == "SECURITY" || new_mode == "DOCKING");

  // Lock if entering SAFE_STOP or FAULT
  if (new_mode == "SAFE_STOP" || new_mode == "FAULT") {
    mode_lock_ = true;
  } else if (new_mode == "READY") {
    // Unlock when explicitly returning to READY
    mode_lock_ = false;
  }

  RCLCPP_INFO(this->get_logger(), "Mode: %s -> %s (%s)",
              previous_mode_.c_str(), new_mode.c_str(), reason.c_str());
}

void ModeManagerNode::publishMode() {
  // Broadcast current mode.
  puppy_interfaces::msg::RobotMode msg;
  msg.header.stamp = this->now();
  msg.current_mode = current_mode_;
  msg.previous_mode = previous_mode_;
  msg.motion_enabled = motion_enabled_;
  msg.autonomy_enabled = autonomy_enabled_;
  msg.reason = "";
  mode_pub_->publish(msg);
}

}  // namespace puppy_core
