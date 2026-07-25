// Puppy 机械狗 - 电池仿真节点（C++ 版本）
//
// 功能：
//   1. 模拟电池电量消耗（移动时消耗更快，空闲时慢消耗）
//   2. 检测机器人是否在充电桩附近，自动充电
//   3. 发布 /battery_state 话题（sensor_msgs/BatteryState）
//   4. 低电量时发布警报
//
// 充电桩位置：(0.0, -2.0)，与 emergency_response / patrol 保持一致
// 仿真速率说明：
//   所有速率均为仿真秒级百分比，适合测试：
//   - 空闲消耗: 0.1%/秒（约17分钟耗尽）
//   - 移动消耗: 0.3%/秒（约5.5分钟耗尽）
//   - 充电速率: 1.0%/秒（约100秒充满到100%）
#pragma once

#include <memory>
#include <optional>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "puppy_interfaces/msg/battery_status.hpp"

namespace puppy_gait {

/// 电池仿真节点
class BatterySimulatorNode : public rclcpp::Node {
 public:
  BatterySimulatorNode();
  ~BatterySimulatorNode() override = default;

 private:
  // AMCL 位姿回调：更新机器人位置并判断是否移动
  void poseCallback(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  // 定时更新电池电量并发布状态
  void updateBattery();
  // 发布电池状态（兼容 topic + 语义消息）
  void publishBatteryState();

 private:
  // ===== 仿真参数 =====
  bool publish_semantic_battery_{true};
  double drain_rate_idle_{0.001};     // 空闲: 0.1%/秒
  double drain_rate_active_{0.003};   // 移动: 0.3%/秒
  double charge_rate_{0.01};          // 充电: 1.0%/秒
  double voltage_full_{12.6};
  double voltage_empty_{10.5};
  double low_battery_threshold_{0.2};  // 低电量阈值 20%

  // ===== 充电桩位置（从 navigation_targets.yaml 加载，默认值）=====
  double dock_x_{0.0};
  double dock_y_{-2.0};
  double dock_range_{1.0};

  // ===== 状态 =====
  double battery_level_{1.0};
  bool is_charging_{false};
  bool is_moving_{false};
  double robot_x_{1.0};
  double robot_y_{-2.0};
  std::optional<std::pair<double, double>> last_position_;
  rclcpp::Time last_time_;
  bool low_battery_alerted_{false};
  int last_logged_percent_{-1};

  // ===== 发布器 =====
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Publisher<puppy_interfaces::msg::BatteryStatus>::SharedPtr
      battery_semantic_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr low_battery_pub_;

  // ===== 订阅器 =====
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      pose_sub_;

  // ===== 定时器（5Hz 更新电池状态）=====
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace puppy_gait
