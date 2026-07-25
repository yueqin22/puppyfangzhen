// Puppy 机械狗 - 里程计 TF 发布器
//
// 订阅 /odom 话题，发布 odom -> base_footprint 变换
// （CoppeliaSim 的 simROS2 不支持 tf2_msgs/msg/TFMessage，所以用此节点转发）
// 注意：simROS2 使用 best_effort QoS，需要匹配
#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace puppy_bringup {

/// 里程计 TF 发布器：订阅 /odom，广播 odom -> base_footprint
class OdomTfPublisherNode : public rclcpp::Node {
 public:
  OdomTfPublisherNode();
  ~OdomTfPublisherNode() override = default;

 private:
  // /odom 回调：发布 odom -> base_footprint 变换
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);

 private:
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

}  // namespace puppy_bringup
