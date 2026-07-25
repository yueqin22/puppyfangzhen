// Adapt legacy /fall_detected Bool topic into semantic FallEvent.
//
// 将旧的 /fall_detected (Bool) 信号转换为 puppy_interfaces/FallEvent。
#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "puppy_interfaces/msg/fall_event.hpp"

namespace puppy_core {

/// Convert legacy Bool fall signal into puppy_interfaces/FallEvent.
class FallEventAdapterNode : public rclcpp::Node {
 public:
  FallEventAdapterNode();
  ~FallEventAdapterNode() override = default;

 private:
  void onFallDetected(const std_msgs::msg::Bool::SharedPtr msg);

 private:
  rclcpp::Publisher<puppy_interfaces::msg::FallEvent>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
};

}  // namespace puppy_core
