// 时钟发布节点实现
//
// use_sim_time=false，本节点用墙钟运行
// simROS2 使用 best_effort QoS，需要匹配
#include "puppy_bringup/clock_publisher.h"

namespace puppy_bringup {

ClockPublisherNode::ClockPublisherNode()
    : rclcpp::Node("clock_publisher") {
  // use_sim_time=false，本节点用墙钟运行
  clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

  // simROS2 使用 best_effort QoS，需要匹配
  rclcpp::QoS qos(10);
  qos.best_effort();
  qos.keep_last(10);

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", qos,
      std::bind(&ClockPublisherNode::onOdom, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(),
              "时钟发布节点已启动（从 /odom 发布 /clock，best_effort QoS）");
}

void ClockPublisherNode::onOdom(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  // 从 /odom 获取仿真时间戳，发布 /clock
  rosgraph_msgs::msg::Clock clock_msg;
  clock_msg.clock = msg->header.stamp;
  clock_pub_->publish(clock_msg);
}

}  // namespace puppy_bringup
