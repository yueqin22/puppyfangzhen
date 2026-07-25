// Fake Status Server: 仿真 PuppyPi 电池与健康状态 (C++ 版本)
//
// 对应 Python: puppypi_mock/fake_status_server.py
#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "puppy_interfaces/msg/robot_health.hpp"

namespace puppypi_mock {

/// 仿真 PuppyPi 电池、关节、健康状态节点
class FakeStatusServerNode : public rclcpp::Node {
 public:
  FakeStatusServerNode();
  ~FakeStatusServerNode() override = default;

 private:
  // 状态发布（20Hz）
  void publishStatus();

 private:
  // ===== 电池状态 =====
  double battery_percent_{0.85};
  double battery_drain_rate_{0.0001};  // 每个周期消耗

  // ===== 发布器 =====
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<puppy_interfaces::msg::RobotHealth>::SharedPtr health_pub_;

  // ===== 定时器（20Hz）=====
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace puppypi_mock
