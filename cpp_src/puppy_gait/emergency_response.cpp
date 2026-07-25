// 紧急联动响应节点实现
#include "puppy_gait/emergency_response.h"

#include <chrono>
#include <cmath>
#include <sstream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace puppy_gait {

EmergencyLocation EmergencyResponseNode::getEmergencyLocation(
    const std::string &event_type) {
  // 紧急事件位置定义
  if (event_type == "fall") {
    return {1.0, -2.0, 0.0, "客厅"};
  } else if (event_type == "gas_leak") {
    return {3.0, 2.0, 0.0, "厨房"};
  } else if (event_type == "intrusion") {
    return {0.0, -1.0, M_PI / 2, "门口"};
  } else if (event_type == "charging") {
    return {0.0, -2.0, M_PI, "充电桩"};
  }
  // 默认返回 fall 位置
  return {1.0, -2.0, 0.0, "客厅"};
}

EmergencyResponseNode::EmergencyResponseNode()
    : rclcpp::Node("emergency_response") {
  // 使用 ReentrantCallbackGroup 以允许回调内调用动作
  cb_group_ = this->create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);

  // 导航客户端
  nav_client_ = rclcpp_action::create_client<NavigateToPose>(
      this, "navigate_to_pose", cb_group_);

  // 状态
  emergency_active_ = false;
  last_fall_state_ = false;
  last_intrusion_ = false;
  last_low_battery_ = false;
  retry_count_ = 0;
  max_retries_ = 5;

  // 订阅器（使用回调组）
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = cb_group_;

  fall_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/fall_detected", 10,
      std::bind(&EmergencyResponseNode::fallCallback, this,
                std::placeholders::_1),
      sub_opts);
  security_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/security/alert", 10,
      std::bind(&EmergencyResponseNode::securityCallback, this,
                std::placeholders::_1),
      sub_opts);
  intrusion_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/security/intrusion", 10,
      std::bind(&EmergencyResponseNode::intrusionCallback, this,
                std::placeholders::_1),
      sub_opts);
  battery_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/low_battery_alert", 10,
      std::bind(&EmergencyResponseNode::batteryCallback, this,
                std::placeholders::_1),
      sub_opts);
  cancel_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/emergency/cancel", 10,
      std::bind(&EmergencyResponseNode::cancelCallback, this,
                std::placeholders::_1),
      sub_opts);

  // 发布器
  voice_pub_ = this->create_publisher<std_msgs::msg::String>("/voice/tts", 10);
  status_pub_ =
      this->create_publisher<std_msgs::msg::String>("/emergency/status", 10);
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // 定时发布状态
  status_timer_ = this->create_wall_timer(
      std::chrono::seconds(5), std::bind(&EmergencyResponseNode::publishStatus, this));

  RCLCPP_INFO(this->get_logger(), "紧急联动响应节点已启动");
  RCLCPP_INFO(this->get_logger(),
              "监听: /fall_detected, /security/alert, /security/intrusion, "
              "/low_battery_alert");
}

void EmergencyResponseNode::fallCallback(
    const std_msgs::msg::Bool::SharedPtr msg) {
  if (msg->data && !last_fall_state_) {
    RCLCPP_WARN(this->get_logger(), "!!! 紧急事件: 检测到跌倒 !!!");
    triggerEmergency("fall", "检测到主人跌倒！正在前往现场确认");
  }
  last_fall_state_ = msg->data;
}

void EmergencyResponseNode::securityCallback(
    const std_msgs::msg::String::SharedPtr msg) {
  const std::string &alert_text = msg->data;
  if (alert_text.find("高级警报") != std::string::npos &&
      alert_text.find("煤气") != std::string::npos) {
    if (!emergency_active_) {
      triggerEmergency("gas_leak",
                       "煤气泄漏警报！" + alert_text + "，正在前往厨房确认");
    }
  } else if (alert_text.find("高级警报") != std::string::npos &&
             (alert_text.find("入侵") != std::string::npos ||
              alert_text.find("闯入") != std::string::npos)) {
    if (!emergency_active_) {
      triggerEmergency("intrusion",
                       "入侵警报！" + alert_text + "，正在前往门口查看");
    }
  }
}

void EmergencyResponseNode::intrusionCallback(
    const std_msgs::msg::Bool::SharedPtr msg) {
  if (msg->data && !last_intrusion_) {
    if (!emergency_active_) {
      triggerEmergency("intrusion", "检测到入侵！正在前往门口查看");
    }
  }
  last_intrusion_ = msg->data;
}

void EmergencyResponseNode::batteryCallback(
    const std_msgs::msg::Bool::SharedPtr msg) {
  if (msg->data && !last_low_battery_) {
    if (!emergency_active_) {
      triggerEmergency("charging", "电量不足，正在自动返回充电桩");
    }
  }
  last_low_battery_ = msg->data;
}

void EmergencyResponseNode::triggerEmergency(
    const std::string &event_type, const std::string &voice_msg) {
  if (emergency_active_) {
    RCLCPP_WARN(this->get_logger(),
                "已有紧急事件进行中，忽略新事件: %s", event_type.c_str());
    return;
  }

  emergency_active_ = true;
  current_emergency_ = event_type;
  retry_count_ = 0;
  EmergencyLocation location = getEmergencyLocation(event_type);
  pending_location_ = location;

  RCLCPP_WARN(this->get_logger(),
              "===== 紧急响应启动: %s → %s =====", event_type.c_str(),
              location.name.c_str());

  publishVoice(voice_msg);
  navigateTo(location.x, location.y, location.yaw);
}

void EmergencyResponseNode::navigateTo(double x, double y, double yaw) {
  if (retry_timer_) {
    retry_timer_->cancel();
    retry_timer_.reset();
  }

  if (!nav_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(this->get_logger(), "Nav2 服务器不可用，3秒后重试...");
    scheduleRetry();
    return;
  }

  NavigateToPose::Goal goal_msg;
  goal_msg.pose.header.stamp = this->now();
  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.pose.position.x = x;
  goal_msg.pose.pose.position.y = y;
  goal_msg.pose.pose.position.z = 0.0;
  goal_msg.pose.pose.orientation.z = std::sin(yaw / 2.0);
  goal_msg.pose.pose.orientation.w = std::cos(yaw / 2.0);

  RCLCPP_INFO(this->get_logger(),
              "发送导航目标: (%.1f, %.1f, yaw=%.2f) [尝试 %d/%d]", x, y, yaw,
              retry_count_ + 1, max_retries_);

  auto send_goal_options =
      rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      std::bind(&EmergencyResponseNode::goalResponseCallback, this,
                std::placeholders::_1);
  send_goal_options.result_callback =
      std::bind(&EmergencyResponseNode::resultCallback, this,
                std::placeholders::_1);

  nav_client_->async_send_goal(goal_msg, send_goal_options);
}

void EmergencyResponseNode::goalResponseCallback(
    std::shared_future<rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr>
        future) {
  // 导航目标响应：与 Python 版一致，拒绝则重试，接受则记录 goal_handle
  auto goal_handle = future.get();
  if (!goal_handle) {
    RCLCPP_WARN(this->get_logger(), "紧急导航目标被拒绝，可能Nav2未就绪");
    scheduleRetry();
    return;
  }
  RCLCPP_INFO(this->get_logger(), "紧急导航目标已接受");
  goal_handle_ = goal_handle;
  // 结果由 result_callback 处理
}

void EmergencyResponseNode::scheduleRetry() {
  if (retry_count_ >= max_retries_) {
    RCLCPP_ERROR(this->get_logger(), "导航重试%d次均失败，放弃紧急导航",
                 max_retries_);
    emergency_active_ = false;
    current_emergency_.clear();
    pending_location_.reset();
    return;
  }
  retry_count_++;
  RCLCPP_INFO(this->get_logger(), "3秒后重试导航 (第%d次)...", retry_count_);
  retry_timer_ = this->create_wall_timer(
      std::chrono::seconds(3),
      std::bind(&EmergencyResponseNode::doRetry, this));
}

void EmergencyResponseNode::doRetry() {
  if (retry_timer_) {
    retry_timer_->cancel();
    retry_timer_.reset();
  }
  if (!pending_location_.has_value() || !emergency_active_) {
    return;
  }
  const auto &loc = pending_location_.value();
  navigateTo(loc.x, loc.y, loc.yaw);
}

void EmergencyResponseNode::resultCallback(
    const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult &
        result) {
  goal_handle_.reset();
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(this->get_logger(), "已到达紧急事件位置: %s",
                current_emergency_.c_str());
    announceArrival();
    cooldown_timer_ = this->create_wall_timer(
        std::chrono::seconds(10),
        std::bind(&EmergencyResponseNode::clearEmergency, this));
  } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
    RCLCPP_WARN(this->get_logger(), "导航中止，执行到达处理");
    announceArrival();
    cooldown_timer_ = this->create_wall_timer(
        std::chrono::seconds(10),
        std::bind(&EmergencyResponseNode::clearEmergency, this));
  } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
    RCLCPP_INFO(this->get_logger(), "导航已取消: %s",
                current_emergency_.c_str());
    emergency_active_ = false;
    current_emergency_.clear();
    pending_location_.reset();
  } else {
    RCLCPP_WARN(this->get_logger(), "导航失败，状态: %d",
                 static_cast<int>(result.code));
    scheduleRetry();
  }
}

void EmergencyResponseNode::announceArrival() {
  // 到达后语音播报
  if (current_emergency_ == "fall") {
    publishVoice("主人，您还好吗？我已经到达现场，正在为您呼叫帮助");
  } else if (current_emergency_ == "gas_leak") {
    publishVoice("已到达厨房，请立即检查煤气阀门，打开窗户通风，不要使用明火");
  } else if (current_emergency_ == "intrusion") {
    publishVoice("已到达门口，未发现异常情况，请主人查看监控确认");
  } else if (current_emergency_ == "charging") {
    publishVoice("已到达充电桩，开始充电");
  }
}

void EmergencyResponseNode::clearEmergency() {
  if (cooldown_timer_) {
    cooldown_timer_->cancel();
    cooldown_timer_.reset();
  }
  RCLCPP_INFO(this->get_logger(), "紧急事件已解除: %s",
              current_emergency_.c_str());
  publishVoice("紧急事件已解除，恢复正常巡逻");
  emergency_active_ = false;
  current_emergency_.clear();
  pending_location_.reset();
}

void EmergencyResponseNode::cancelCallback(
    const std_msgs::msg::String::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "手动取消紧急事件: %s", msg->data.c_str());
  if (cooldown_timer_) {
    cooldown_timer_->cancel();
    cooldown_timer_.reset();
  }
  if (retry_timer_) {
    retry_timer_->cancel();
    retry_timer_.reset();
  }
  if (goal_handle_) {
    RCLCPP_INFO(this->get_logger(), "取消进行中的导航目标");
    auto cancel_future = nav_client_->async_cancel_goal(goal_handle_);
    (void)cancel_future;
    goal_handle_.reset();
  }
  emergency_active_ = false;
  current_emergency_.clear();
  pending_location_.reset();
  geometry_msgs::msg::Twist stop;
  cmd_vel_pub_->publish(stop);
}

void EmergencyResponseNode::cancelDoneCallback(
    rclcpp_action::Client<NavigateToPose>::CancelResponse::SharedPtr
        cancel_response) {
  // 与 Python 版逻辑一致：检查 goals_canceling 是否非空
  if (cancel_response && !cancel_response->goals_canceling.empty()) {
    RCLCPP_INFO(this->get_logger(), "导航目标已成功取消");
  } else {
    RCLCPP_WARN(this->get_logger(), "导航目标取消失败");
  }
}

void EmergencyResponseNode::publishStatus() {
  // 发布状态 (JSON)
  std::ostringstream oss;
  oss << "{\"active\":" << (emergency_active_ ? "true" : "false")
      << ",\"event\":\""
      << (current_emergency_.empty() ? "none" : current_emergency_)
      << "\"}";
  std_msgs::msg::String status;
  status.data = oss.str();
  status_pub_->publish(status);
}

void EmergencyResponseNode::publishVoice(const std::string &text) {
  std_msgs::msg::String msg;
  msg.data = text;
  voice_pub_->publish(msg);
}

}  // namespace puppy_gait
