// Battery Status Adapter 实现
#include "puppy_core/battery_status_adapter.h"

#include <algorithm>

namespace puppy_core {

BatteryStatusAdapterNode::BatteryStatusAdapterNode()
    : rclcpp::Node("battery_status_adapter") {
  this->declare_parameter("low_battery_threshold", 0.20);
  this->declare_parameter("critical_battery_threshold", 0.10);
  low_battery_threshold_ =
      this->get_parameter("low_battery_threshold").as_double();
  critical_battery_threshold_ =
      this->get_parameter("critical_battery_threshold").as_double();

  pub_ = this->create_publisher<puppy_interfaces::msg::BatteryStatus>(
      "/battery_status", 10);
  sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
      "/battery_state", 10,
      std::bind(&BatteryStatusAdapterNode::onBattery, this,
                std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Battery status adapter started");
}

void BatteryStatusAdapterNode::onBattery(
    const sensor_msgs::msg::BatteryState::SharedPtr msg) {
  puppy_interfaces::msg::BatteryStatus status;
  status.header = msg->header;
  status.voltage = msg->voltage;
  status.current = msg->current;
  status.percent = static_cast<float>(
      std::max(0.0, std::min(1.0, static_cast<double>(msg->percentage))));
  // Python 版使用 power_supply_status in (CHARGING, FULL)
  using BS = sensor_msgs::msg::BatteryState;
  status.charging = (msg->power_supply_status == BS::POWER_SUPPLY_STATUS_CHARGING) ||
                    (msg->power_supply_status == BS::POWER_SUPPLY_STATUS_FULL);
  status.low_battery = static_cast<double>(status.percent) <= low_battery_threshold_;
  status.critical_battery =
      static_cast<double>(status.percent) <= critical_battery_threshold_;
  pub_->publish(status);
}

}  // namespace puppy_core
