// 简单巡逻脚本 - 家庭环境版 (C++ 版本)
//
// 使用 Nav2 NavigateToPose action client 依次导航到预设航点，循环巡逻。
// 对应 Python: puppy_nav/scripts/simple_patrol.py
#pragma once

#include <chrono>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

namespace puppy_nav {

/// 巡逻机器人节点：使用 Nav2 action client 依次导航到预设航点
class PatrolBot : public rclcpp::Node {
 public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;

  PatrolBot();
  ~PatrolBot() override = default;

  /// 导航到指定位置
  /// @param x 目标 x 坐标
  /// @param y 目标 y 坐标
  /// @param yaw 目标朝向（弧度）
  /// @param timeout 超时时间（秒）
  /// @return 是否成功到达
  bool navigateTo(double x, double y, double yaw = 0.0,
                  double timeout = 60.0);

 private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
};

}  // namespace puppy_nav
