// 老人跌倒检测节点实现
#include "puppy_gait/fall_detection.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>

namespace puppy_gait {

FallDetectionNode::FallDetectionNode() : rclcpp::Node("fall_detection") {
  // ===== 参数声明 =====
  this->declare_parameter("auto_simulate_falls", false);
  this->declare_parameter("sim_cycle_seconds", 60.0);
  this->declare_parameter("normal_height", 1.7);
  this->declare_parameter("fallen_height", 0.3);
  this->declare_parameter("height_drop_threshold", 0.5);
  this->declare_parameter("fall_confirm_duration", 1.0);
  this->declare_parameter("alert_cooldown", 30.0);

  auto_sim_ = this->get_parameter("auto_simulate_falls").as_bool();

  // P1-2: 跌倒检测阈值从参数读取（可由 features.yaml 覆盖）
  normal_height_ = this->get_parameter("normal_height").as_double();
  fallen_height_ = this->get_parameter("fallen_height").as_double();
  height_drop_threshold_ = this->get_parameter("height_drop_threshold").as_double();
  fall_confirm_duration_ = this->get_parameter("fall_confirm_duration").as_double();
  alert_cooldown_ = this->get_parameter("alert_cooldown").as_double();

  // ===== 订阅器 =====
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/depth_camera/image_raw", 10,
      std::bind(&FallDetectionNode::imageCallback, this, std::placeholders::_1));

  // ===== 发布器 =====
  fall_pub_ = this->create_publisher<std_msgs::msg::Bool>("/fall_detected", 10);
  // P0-1: 发布语义消息 FallEvent 到 /fall/event
  // 供 puppy_core（mode_manager/safety_manager）订阅
  // 保留 Bool /fall_detected 作为兼容 topic（security_node 订阅）
  fall_event_pub_ =
      this->create_publisher<puppy_interfaces::msg::FallEvent>("/fall/event", 10);
  status_pub_ =
      this->create_publisher<std_msgs::msg::String>("/fall_detection/status", 10);

  // ===== 状态初始化 =====
  current_height_ = normal_height_;
  previous_height_ = normal_height_;

  // ===== 服务 =====
  test_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/fall_detection/trigger_test",
      std::bind(&FallDetectionNode::triggerTestCallback, this,
                std::placeholders::_1, std::placeholders::_2));
  recover_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/fall_detection/trigger_recover",
      std::bind(&FallDetectionNode::triggerRecoverCallback, this,
                std::placeholders::_1, std::placeholders::_2));

  // ===== 定时器 =====
  status_timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&FallDetectionNode::publishStatus, this));

  if (auto_sim_) {
    double sim_cycle = this->get_parameter("sim_cycle_seconds").as_double();
    sim_cycle_frames_ = static_cast<int>(sim_cycle / 0.1);
    sim_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&FallDetectionNode::simulationStep, this));
    RCLCPP_INFO(this->get_logger(),
                "  自动模拟跌倒已开启（周期 %.1fs）", sim_cycle);
  } else {
    RCLCPP_INFO(this->get_logger(), "  自动模拟跌倒已关闭（默认模式）");
    RCLCPP_INFO(this->get_logger(),
                "  测试跌倒: ros2 service call /fall_detection/trigger_test "
                "std_srvs/srv/Trigger");
  }

  RCLCPP_INFO(this->get_logger(), "%s", "==================================================");
  RCLCPP_INFO(this->get_logger(), "  老人跌倒检测节点启动（仿真版本）");
  RCLCPP_INFO(this->get_logger(), "  正常站立高度: %.2f m", normal_height_);
  RCLCPP_INFO(this->get_logger(),
              "  跌倒判定高度变化阈值: %.0f%%",
              height_drop_threshold_ * 100.0);
  RCLCPP_INFO(this->get_logger(),
              "  发布话题: /fall_detected, /fall_detection/status");
  RCLCPP_INFO(this->get_logger(), "%s", "==================================================");

  publishStatusMsg("节点启动，等待检测...");
}

void FallDetectionNode::simulationStep() {
  if (!auto_sim_) {
    return;
  }
  frame_count_++;
  previous_height_ = current_height_;
  sim_phase_ += 0.05;

  int cycle = frame_count_ % sim_cycle_frames_;
  int fall_start = sim_cycle_frames_ / 2;
  int fall_end = fall_start + 50;
  if (cycle >= fall_start && cycle <= fall_end) {
    if (cycle == fall_start) {
      RCLCPP_WARN(this->get_logger(),
                  "仿真事件: 检测到人体高度突然降低（模拟跌倒）");
    }
    current_height_ = fallen_height_ + 0.05 * std::sin(sim_phase_);
  } else {
    current_height_ = normal_height_ + 0.05 * std::sin(sim_phase_);
  }

  detectFall();
}

void FallDetectionNode::triggerTestCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  RCLCPP_WARN(this->get_logger(), "[测试] 手动触发跌倒事件！");
  previous_height_ = normal_height_;
  current_height_ = fallen_height_;
  triggerFallAlert();
  response->success = true;
  response->message = "跌倒测试事件已触发";
}

void FallDetectionNode::triggerRecoverCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  RCLCPP_INFO(this->get_logger(), "[测试] 手动触发恢复事件！");
  current_height_ = normal_height_;
  is_fallen_ = false;
  fall_start_time_.reset();
  last_alert_time_ = this->now();
  std_msgs::msg::Bool fall_msg;
  fall_msg.data = false;
  fall_pub_->publish(fall_msg);
  publishFallEvent(false, "", 0.0, "manual_recover");
  publishStatusMsg("检测到老人已站起，跌倒状态解除");
  response->success = true;
  response->message = "恢复事件已触发";
}

void FallDetectionNode::imageCallback(
    const sensor_msgs::msg::Image::SharedPtr /*msg*/) {
  // 仿真版本不实际处理图像
}

void FallDetectionNode::detectFall() {
  rclcpp::Time now = this->now();

  double height_change_ratio = 0.0;
  if (previous_height_ > 0.01) {
    height_change_ratio =
        (previous_height_ - current_height_) / previous_height_;
  }

  if (height_change_ratio > height_drop_threshold_) {
    if (!fall_start_time_.has_value()) {
      fall_start_time_ = now;
      char buf[128];
      std::snprintf(buf, sizeof(buf),
                    "疑似跌倒：高度从 %.2fm 降至 %.2fm",
                    previous_height_, current_height_);
      publishStatusMsg(buf);
    }

    double elapsed = (now - fall_start_time_.value()).seconds();
    if (elapsed >= fall_confirm_duration_ && !is_fallen_) {
      triggerFallAlert();
    }
  } else {
    if (fall_start_time_.has_value() && !is_fallen_) {
      publishStatusMsg("高度恢复正常，取消疑似跌倒警报");
    }
    fall_start_time_.reset();
  }

  if (is_fallen_ && current_height_ > normal_height_ * 0.7) {
    is_fallen_ = false;
    fall_start_time_.reset();
    last_alert_time_ = now;
    publishStatusMsg("检测到老人已站起，跌倒状态解除");
    std_msgs::msg::Bool fall_msg;
    fall_msg.data = false;
    fall_pub_->publish(fall_msg);
    publishFallEvent(false, "", 0.0, "auto_recover");
  }
}

void FallDetectionNode::triggerFallAlert() {
  rclcpp::Time now = this->now();

  if (last_alert_time_.has_value()) {
    double elapsed = (now - last_alert_time_.value()).seconds();
    if (elapsed < alert_cooldown_) {
      return;
    }
  }

  is_fallen_ = true;
  last_alert_time_ = now;

  std_msgs::msg::Bool alert_msg;
  alert_msg.data = true;
  fall_pub_->publish(alert_msg);

  // P0-1: 同时发布语义消息 FallEvent
  double confidence = (previous_height_ - current_height_) /
                       std::max(previous_height_, 0.01);
  confidence = std::max(0.0, std::min(1.0, confidence));
  publishFallEvent(true, "down", confidence, "height_drop");

  char buf[128];
  std::snprintf(buf, sizeof(buf),
                "跌倒警报！检测到老人跌倒 (当前高度: %.2fm)",
                current_height_);
  publishStatusMsg(buf);
  RCLCPP_ERROR(this->get_logger(), "%s", buf);
}

void FallDetectionNode::publishStatusMsg(const std::string &message) {
  std_msgs::msg::String status;
  status.data = message;
  status_pub_->publish(status);
}

void FallDetectionNode::publishFallEvent(bool detected,
                                         const std::string &direction,
                                         double confidence,
                                         const std::string &source) {
  // P0-1: 发布语义消息 FallEvent 到 /fall/event。
  // 供 puppy_core（mode_manager/safety_manager）订阅。与 Bool /fall_detected
  // 同时发布，保持向后兼容。
  puppy_interfaces::msg::FallEvent msg;
  msg.header.stamp = this->now();
  msg.detected = detected;
  msg.direction = direction;
  msg.confidence = static_cast<float>(confidence);
  msg.source = source;
  fall_event_pub_->publish(msg);
}

void FallDetectionNode::publishStatus() {
  std::string state_str = is_fallen_ ? "跌倒" : "正常";
  char buf[128];
  std::snprintf(buf, sizeof(buf), "[状态: %s] 当前高度: %.2fm",
                state_str.c_str(), current_height_);
  publishStatusMsg(buf);
}

}  // namespace puppy_gait
