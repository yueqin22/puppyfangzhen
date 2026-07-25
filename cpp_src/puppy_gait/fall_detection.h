// Puppy 机械狗 - 老人跌倒检测节点（C++ 仿真版本）
//
// 功能：
//   1. 订阅 /camera/depth_camera/image_raw 话题（模拟摄像头图像）
//   2. 仿真人体姿态估计，检测老人是否跌倒
//   3. 当检测到人体高度突然降低（跌倒特征）时发布警报
//   4. 发布 /fall_detected 话题（Bool 类型，True 表示检测到跌倒）
//   5. 发布 /fall_detection/status 话题（String 类型，状态信息）
//   6. 提供 /fall_detection/trigger_test 服务手动触发测试跌倒
//
// 注意：这是仿真版本，默认不自动模拟跌倒事件，避免误报。
// 测试跌倒：ros2 service call /fall_detection/trigger_test std_srvs/srv/Trigger
// 测试恢复：等待机器人移动（仿真中高度会"恢复"），或重启节点
// 实际部署时需要接入真实的人体姿态估计模型。
#pragma once

#include <memory>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "puppy_interfaces/msg/fall_event.hpp"

namespace puppy_gait {

/// 老人跌倒检测节点（仿真版本）
class FallDetectionNode : public rclcpp::Node {
 public:
  FallDetectionNode();
  ~FallDetectionNode() override = default;

 private:
  // 仿真步进（自动模拟跌倒时由定时器调用）
  void simulationStep();
  // 跌倒检测核心逻辑
  void detectFall();
  // 触发跌倒警报
  void triggerFallAlert();
  // 发布状态消息
  void publishStatusMsg(const std::string &message);
  // P0-1: 发布语义消息 FallEvent 到 /fall/event
  void publishFallEvent(bool detected, const std::string &direction,
                        double confidence, const std::string &source);
  // 定时发布状态
  void publishStatus();

  // 服务回调
  void triggerTestCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void triggerRecoverCallback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // 图像回调（仿真版本不实际处理图像）
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

 private:
  // ===== 仿真参数 =====
  bool auto_sim_{false};
  int sim_cycle_frames_{600};  // 60s / 0.1s

  // P1-2: 跌倒检测阈值从参数读取（可由 features.yaml 覆盖）
  double normal_height_{1.7};
  double fallen_height_{0.3};
  double height_drop_threshold_{0.5};
  double fall_confirm_duration_{1.0};
  double alert_cooldown_{30.0};

  // ===== 状态 =====
  double current_height_{1.7};
  double previous_height_{1.7};
  std::optional<rclcpp::Time> fall_start_time_;
  std::optional<rclcpp::Time> last_alert_time_;
  bool is_fallen_{false};
  int frame_count_{0};
  double sim_phase_{0.0};

  // ===== 订阅器 =====
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

  // ===== 发布器 =====
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr fall_pub_;
  rclcpp::Publisher<puppy_interfaces::msg::FallEvent>::SharedPtr fall_event_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  // ===== 服务 =====
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr test_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr recover_srv_;

  // ===== 定时器 =====
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr sim_timer_;
};

}  // namespace puppy_gait
