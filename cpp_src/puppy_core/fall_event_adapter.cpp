// Fall Event Adapter 实现
#include "puppy_core/fall_event_adapter.h"

namespace puppy_core {

FallEventAdapterNode::FallEventAdapterNode()
    : rclcpp::Node("fall_event_adapter") {
  pub_ = this->create_publisher<puppy_interfaces::msg::FallEvent>(
      "/fall/event", 10);
  sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/fall_detected", 10,
      std::bind(&FallEventAdapterNode::onFallDetected, this,
                std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Fall event adapter started");
}

void FallEventAdapterNode::onFallDetected(
    const std_msgs::msg::Bool::SharedPtr msg) {
  puppy_interfaces::msg::FallEvent event;
  event.header.stamp = this->now();
  event.detected = msg->data;
  event.direction = "unknown";
  event.confidence = msg->data ? 1.0f : 0.0f;
  event.source = "legacy_bool_adapter";
  pub_->publish(event);
}

}  // namespace puppy_core
