// 自动巡航节点实现
#include "puppy_gait/auto_cruise.h"

#include <chrono>

namespace puppy_gait {

AutoCruiseNode::AutoCruiseNode() : rclcpp::Node("auto_cruise") {
  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // 巡航状态
  state_start_time_ = this->now();

  // 巡航步骤定义
  // 每步: (duration_sec, linear_x, angular_z, description)
  steps_ = {
      // 客厅：原地旋转扫描
      {3.0, 0.0, 0.3, "客厅: 旋转扫描环境"},
      // 客厅：向前走到走廊入口
      {4.0, 0.2, 0.0, "客厅: 向前走向走廊"},
      // 走廊：左转向北
      {3.0, 0.0, 0.4, "走廊入口: 左转"},
      // 走廊：向北走到卧室门口
      {5.0, 0.2, 0.0, "走廊: 向北走"},
      // 卧室：进入卧室
      {3.0, 0.15, 0.0, "卧室: 进入"},
      // 卧室：旋转扫描
      {4.0, 0.0, 0.3, "卧室: 旋转扫描"},
      // 卧室：退出
      {3.0, -0.15, 0.0, "卧室: 后退退出"},
      // 走廊：继续向北到厨房
      {4.0, 0.2, 0.0, "走廊: 继续向北"},
      // 厨房：进入厨房
      {3.0, 0.15, 0.0, "厨房: 进入"},
      // 厨房：旋转扫描
      {4.0, 0.0, -0.3, "厨房: 旋转扫描"},
      // 厨房：退出
      {3.0, -0.15, 0.0, "厨房: 后退退出"},
      // 走廊：向南返回
      {8.0, 0.2, 0.0, "走廊: 向南返回"},
      // 客厅：右转
      {3.0, 0.0, -0.4, "客厅入口: 右转"},
      // 客厅：回到起点
      {4.0, 0.2, 0.0, "客厅: 回到起点"},
      // 客厅：最终旋转扫描
      {5.0, 0.0, 0.3, "客厅: 最终扫描"},
  };

  current_step_ = 0;

  RCLCPP_INFO(this->get_logger(), "=== 自动巡航启动 ===");
  RCLCPP_INFO(this->get_logger(),
              "巡航路线: 客厅 -> 走廊 -> 卧室 -> 走廊 -> 厨房 -> 客厅");
  if (!steps_.empty()) {
    RCLCPP_INFO(this->get_logger(), "步骤 0/%zu: %s", steps_.size(),
                steps_[0].description.c_str());
  }

  // 控制周期 20Hz
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&AutoCruiseNode::timerCallback, this));
}

void AutoCruiseNode::timerCallback() {
  if (current_step_ >= steps_.size()) {
    // 巡航完成，停止
    geometry_msgs::msg::Twist twist;
    cmd_pub_->publish(twist);
    RCLCPP_INFO(this->get_logger(), "=== 自动巡航完成！地图已建好 ===");
    timer_->cancel();
    return;
  }

  const auto &step = steps_[current_step_];
  double elapsed = (this->now() - state_start_time_).seconds();

  if (elapsed >= step.duration_sec) {
    // 进入下一步
    current_step_++;
    state_start_time_ = this->now();
    if (current_step_ < steps_.size()) {
      RCLCPP_INFO(this->get_logger(), "步骤 %zu/%zu: %s", current_step_,
                  steps_.size(), steps_[current_step_].description.c_str());
    }
    // 停顿 0.5 秒
    geometry_msgs::msg::Twist twist;
    cmd_pub_->publish(twist);
    return;
  }

  // 发布速度命令
  geometry_msgs::msg::Twist twist;
  twist.linear.x = step.linear_x;
  twist.angular.z = step.angular_z;
  cmd_pub_->publish(twist);
}

}  // namespace puppy_gait
