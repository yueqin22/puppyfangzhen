// Fake Motion Server: 仿真 PuppyPi 运动平台 (C++ 版本)
//
// 响应 /cmd_vel 和姿态命令，仿真运动执行（带真实延迟），
// 并发布 PlatformMotionState。
// 对应 Python: puppypi_mock/fake_motion_server.py
#pragma once

#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "puppy_interfaces/msg/platform_motion_state.hpp"

namespace puppypi_mock {

/// 仿真 PuppyPi 运动平台节点
class FakeMotionServerNode : public rclcpp::Node {
 public:
  FakeMotionServerNode();
  ~FakeMotionServerNode() override = default;

 private:
  // 模拟自动起立序列（启动后 2 秒触发）
  void autoStand();
  // /cmd_vel_safe 订阅回调
  void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg);
  // /robot/posture_cmd 订阅回调
  void onPosture(const std_msgs::msg::String::SharedPtr msg);
  // 仿真主循环（20Hz）：更新位姿并发布状态
  void updateSim();

 private:
  // ===== 运动状态 =====
  bool standing_{false};
  bool moving_{false};
  bool controllable_{false};
  std::string current_posture_{"SIT"};
  double vx_{0.0};
  double wz_{0.0};

  // ===== 仿真位姿（用于里程计反馈）=====
  double sim_x_{0.0};
  double sim_y_{0.0};
  double sim_yaw_{0.0};

  // ===== 发布器 =====
  rclcpp::Publisher<puppy_interfaces::msg::PlatformMotionState>::SharedPtr
      state_pub_;

  // ===== 订阅器 =====
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr posture_sub_;

  // ===== 定时器 =====
  rclcpp::TimerBase::SharedPtr sim_timer_;       // 20Hz 仿真循环
  rclcpp::TimerBase::SharedPtr auto_stand_timer_;  // 2 秒后自动起立
};

}  // namespace puppypi_mock
