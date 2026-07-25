// Manual /clock publisher for Gazebo simulation (C++ 版本)
//
// 从 /odom 话题读取时间戳（Gazebo 发布，使用仿真时间），
// 并重新发布到 /clock，使 use_sim_time=true 的节点能正确同步。
//
// 之所以需要这个节点，是因为当前环境中的 gazebo_ros 不会自动发布 /clock。
//
// 对应 Python: puppy_worlds/scripts/clock_publisher.py
#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "builtin_interfaces/msg/time.hpp"

namespace puppy_worlds {

/// 时钟发布节点：从 /odom 提取仿真时间并发布到 /clock
class ClockPublisher : public rclcpp::Node {
 public:
  ClockPublisher();
  ~ClockPublisher() override = default;

 private:
  // /odom 订阅回调：提取仿真时间并发布到 /clock
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

 private:
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // 最近一次发布的时间（对应 Python: self.last_time）
  builtin_interfaces::msg::Time last_time_;
  bool has_last_time_{false};
};

}  // namespace puppy_worlds
