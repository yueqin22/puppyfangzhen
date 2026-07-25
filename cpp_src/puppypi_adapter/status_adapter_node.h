// Status Adapter Node: 将 PuppyPi 状态转换为 ROS2 消息
//
// 输入：PuppyPi 板载反馈（电机、姿态、错误）
// 输出：/platform/health, /joint_states, /battery_state (原始)
//       /battery_status (语义，可选)
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "puppy_interfaces/msg/battery_status.hpp"
#include "puppy_interfaces/msg/robot_health.hpp"

namespace puppypi_adapter {

/// 将 PuppyPi 平台状态转换为标准 ROS2 消息
class StatusAdapterNode : public rclcpp::Node {
 public:
  StatusAdapterNode();
  ~StatusAdapterNode() override = default;

 private:
  // 轮询 PuppyPi 状态并发布到 ROS2
  void pollStatus();

 private:
  // ===== 参数 =====
  bool use_sim_{true};
  bool publish_raw_battery_{true};
  bool publish_semantic_battery_{true};
  double battery_drain_per_tick_{0.0002};

  // ===== 状态 =====
  double battery_percent_{0.80};
  bool sdk_connected_{false};
  std::vector<std::string> joint_names_;

  // ===== 发布器 =====
  rclcpp::Publisher<puppy_interfaces::msg::RobotHealth>::SharedPtr health_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Publisher<puppy_interfaces::msg::BatteryStatus>::SharedPtr
      battery_semantic_pub_;

  // ===== 定时器（20Hz 轮询）=====
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace puppypi_adapter
