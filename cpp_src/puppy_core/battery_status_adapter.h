// Adapt raw BatteryState into semantic BatteryStatus for puppy_core.
//
// 将 sensor_msgs/BatteryState 转换为 puppy_interfaces/BatteryStatus。
#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "puppy_interfaces/msg/battery_status.hpp"

namespace puppy_core {

/// Convert sensor_msgs/BatteryState into puppy_interfaces/BatteryStatus.
class BatteryStatusAdapterNode : public rclcpp::Node {
 public:
  BatteryStatusAdapterNode();
  ~BatteryStatusAdapterNode() override = default;

 private:
  void onBattery(const sensor_msgs::msg::BatteryState::SharedPtr msg);

 private:
  double low_battery_threshold_{0.20};
  double critical_battery_threshold_{0.10};

  rclcpp::Publisher<puppy_interfaces::msg::BatteryStatus>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr sub_;
};

}  // namespace puppy_core
