// 紧急联动响应节点（C++ 版本）
//
// - 跌倒检测 → 机器人导航到跌倒位置 + 语音紧急呼叫
// - 煤气泄漏 → 机器人导航到厨房确认 + 语音警报
// - 入侵检测 → 机器人导航到入侵位置 + 语音警告
// - 低电量 → 自动返回充电桩
//
// 联动策略:
//   1. 接收紧急事件
//   2. 取消当前导航任务
//   3. 导航到事件位置（失败则重试）
//   4. 语音播报紧急信息
//   5. 等待事件解除
#pragma once

#include <memory>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

namespace puppy_gait {

/// 紧急事件位置定义
struct EmergencyLocation {
  double x;
  double y;
  double yaw;
  std::string name;
};

/// 紧急联动响应节点
class EmergencyResponseNode : public rclcpp::Node {
 public:
  EmergencyResponseNode();
  ~EmergencyResponseNode() override = default;

 private:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;

  // ===== 回调 =====
  void fallCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void securityCallback(const std_msgs::msg::String::SharedPtr msg);
  void intrusionCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void batteryCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void cancelCallback(const std_msgs::msg::String::SharedPtr msg);

  // 触发紧急事件
  void triggerEmergency(const std::string &event_type,
                        const std::string &voice_msg);
  // 导航到目标
  void navigateTo(double x, double y, double yaw);
  // 调度重试
  void scheduleRetry();
  // 执行重试
  void doRetry();
  // 到达后语音播报
  void announceArrival();
  // 清除紧急状态
  void clearEmergency();
  // 定时发布状态
  void publishStatus();
  // 发布语音
  void publishVoice(const std::string &text);

  // Action 回调
  void goalResponseCallback(
      std::shared_future<rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr>
          future);
  void resultCallback(
      const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult &
          result);
  void cancelDoneCallback(
      rclcpp_action::Client<NavigateToPose>::CancelResponse::SharedPtr
          cancel_response);

  // 根据 event_type 获取紧急位置
  static EmergencyLocation getEmergencyLocation(const std::string &event_type);

 private:
  // 使用 ReentrantCallbackGroup 以允许回调内调用动作
  rclcpp::CallbackGroup::SharedPtr cb_group_;

  // 导航客户端
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;

  // 发布器
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr voice_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  // 订阅器
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fall_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr security_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr intrusion_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr battery_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cancel_sub_;

  // 状态
  std::string current_emergency_;
  bool emergency_active_{false};
  bool last_fall_state_{false};
  bool last_intrusion_{false};
  bool last_low_battery_{false};
  rclcpp::TimerBase::SharedPtr cooldown_timer_;
  rclcpp::TimerBase::SharedPtr retry_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  std::shared_ptr<rclcpp_action::ClientGoalHandle<NavigateToPose>>
      goal_handle_;
  std::optional<EmergencyLocation> pending_location_;
  int retry_count_{0};
  int max_retries_{5};
};

}  // namespace puppy_gait
