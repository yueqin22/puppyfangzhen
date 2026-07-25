// 里程计 TF 发布器实现
#include "puppy_bringup/odom_tf_publisher.h"

namespace puppy_bringup {

OdomTfPublisherNode::OdomTfPublisherNode()
    : rclcpp::Node("odom_tf_publisher") {
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  // simROS2 使用 best_effort QoS，需要匹配
  rclcpp::QoS qos(10);
  qos.best_effort();
  qos.keep_last(10);

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", qos,
      std::bind(&OdomTfPublisherNode::onOdom, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(),
              "里程计 TF 发布器已启动，订阅 /odom (best_effort QoS)");
}

void OdomTfPublisherNode::onOdom(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = msg->header.stamp;
  t.header.frame_id = msg->header.frame_id;    // odom
  t.child_frame_id = msg->child_frame_id;      // base_footprint
  t.transform.translation.x = msg->pose.pose.position.x;
  t.transform.translation.y = msg->pose.pose.position.y;
  t.transform.translation.z = msg->pose.pose.position.z;
  t.transform.rotation = msg->pose.pose.orientation;
  tf_broadcaster_->sendTransform(t);
}

}  // namespace puppy_bringup
