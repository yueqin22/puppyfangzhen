// 时钟发布节点 - 从 /odom 时间戳发布 /clock
//
// 关键：simROS2 插件不支持 rosgraph_msgs/msg/Clock，需要单独节点发布
// use_sim_time=false（本节点用墙钟运行），从 /odom 获取仿真时间
// 注意：simROS2 使用 best_effort QoS，需要匹配
#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

namespace puppy_bringup {

/// 时钟发布节点：从 /odom 时间戳发布 /clock
class ClockPublisherNode : public rclcpp::Node {
 public:
  ClockPublisherNode();
  ~ClockPublisherNode() override = default;

 private:
  // /odom 回调：从里程计时间戳提取仿真时间并发布 /clock
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);

 private:
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

}  // namespace puppy_bringup
