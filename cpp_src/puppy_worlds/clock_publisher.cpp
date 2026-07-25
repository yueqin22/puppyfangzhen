// Manual /clock publisher 实现
//
// 从 /odom 提取仿真时间并发布到 /clock
#include "puppy_worlds/clock_publisher.h"

namespace puppy_worlds {

ClockPublisher::ClockPublisher() : rclcpp::Node("clock_publisher") {
  // 等价 Python:
  //   qos_profile = QoSProfile(
  //       reliability=QoSReliabilityPolicy.BEST_EFFORT,
  //       durability=QoSDurabilityPolicy.VOLATILE,
  //       history=QoSHistoryPolicy.KEEP_LAST,
  //       depth=10,
  //   )
  rclcpp::QoS qos_profile(10);  // KEEP_LAST, depth=10
  qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  qos_profile.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

  // /clock 发布器
  clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock",
                                                                 qos_profile);

  // /odom 订阅器
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&ClockPublisher::odomCallback, this, std::placeholders::_1));

  has_last_time_ = false;
  RCLCPP_INFO(this->get_logger(), "Clock publisher started - reading from /odom");
}

void ClockPublisher::odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  // 从里程计消息中提取仿真时间并发布到 /clock
  // 等价 Python:
  //   clock_msg = Clock()
  //   clock_msg.clock = msg.header.stamp
  //   self.clock_pub.publish(clock_msg)
  //   self.last_time = msg.header.stamp
  rosgraph_msgs::msg::Clock clock_msg;
  clock_msg.clock = msg->header.stamp;
  clock_pub_->publish(clock_msg);
  last_time_ = msg->header.stamp;
  has_last_time_ = true;
}

}  // namespace puppy_worlds

// ===== 节点入口 =====
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<puppy_worlds::ClockPublisher>());
  rclcpp::shutdown();
  return 0;
}
