// Fake Motion Server 实现
//
// 仿真 PuppyPi 运动平台
#include "puppypi_mock/fake_motion_server.h"

#include <chrono>
#include <cctype>
#include <cmath>
#include <functional>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace puppypi_mock {

FakeMotionServerNode::FakeMotionServerNode()
    : rclcpp::Node("fake_motion_server") {
  // 运动状态初始化
  standing_ = false;
  moving_ = false;
  controllable_ = false;
  current_posture_ = "SIT";
  vx_ = 0.0;
  wz_ = 0.0;

  // 仿真位姿初始化（用于里程计反馈）
  sim_x_ = 0.0;
  sim_y_ = 0.0;
  sim_yaw_ = 0.0;

  // 发布器
  state_pub_ =
      this->create_publisher<puppy_interfaces::msg::PlatformMotionState>(
          "/platform/motion_state", 10);

  // 订阅器
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_safe", 10,
      std::bind(&FakeMotionServerNode::onCmdVel, this, std::placeholders::_1));
  posture_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/robot/posture_cmd", 10,
      std::bind(&FakeMotionServerNode::onPosture, this, std::placeholders::_1));

  // 仿真定时器（20Hz）
  sim_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&FakeMotionServerNode::updateSim, this));

  // 2 秒后自动起立（仿真启动序列）
  auto_stand_timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&FakeMotionServerNode::autoStand, this));

  RCLCPP_INFO(this->get_logger(),
              "Fake motion server started (will auto-stand in 2s)");
}

void FakeMotionServerNode::autoStand() {
  // 模拟自动起立序列
  if (!standing_) {
    standing_ = true;
    controllable_ = true;
    current_posture_ = "STAND";
    RCLCPP_INFO(this->get_logger(),
                "Fake platform: STOOD UP, controllable=True");
  }
  // 单次触发后取消定时器
  auto_stand_timer_->cancel();
}

void FakeMotionServerNode::onCmdVel(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  if (!controllable_) {
    return;
  }
  vx_ = msg->linear.x;
  wz_ = msg->angular.z;
  moving_ = std::abs(vx_) > 0.01 || std::abs(wz_) > 0.05;
}

void FakeMotionServerNode::onPosture(
    const std_msgs::msg::String::SharedPtr msg) {
  // 转大写
  std::string posture = msg->data;
  for (auto& c : posture) {
    c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  }
  RCLCPP_INFO(this->get_logger(), "Fake posture: %s", posture.c_str());
  current_posture_ = posture;

  if (posture == "STAND") {
    standing_ = true;
    controllable_ = true;
  } else if (posture == "SIT" || posture == "LIE_DOWN" ||
             posture == "FREEZE") {
    standing_ = false;
    controllable_ = false;
    vx_ = 0.0;
    wz_ = 0.0;
  }
}

void FakeMotionServerNode::updateSim() {
  // 仿真运动并发布状态
  if (controllable_ && moving_) {
    const double dt = 0.05;
    sim_x_ += vx_ * std::cos(sim_yaw_) * dt;
    sim_y_ += vx_ * std::sin(sim_yaw_) * dt;
    sim_yaw_ += wz_ * dt;
    // 归一化 yaw 到 [-pi, pi]
    while (sim_yaw_ > M_PI) {
      sim_yaw_ -= 2.0 * M_PI;
    }
    while (sim_yaw_ < -M_PI) {
      sim_yaw_ += 2.0 * M_PI;
    }
  }

  puppy_interfaces::msg::PlatformMotionState msg;
  msg.header.stamp = this->now();
  msg.standing = standing_;
  msg.moving = moving_;
  msg.controllable = controllable_;
  msg.gait_mode = moving_ ? "trot" : "idle";
  msg.linear_x = vx_;
  msg.linear_y = 0.0;
  msg.angular_z = wz_;
  if (moving_) {
    msg.platform_state = "EXECUTING";
  } else if (controllable_) {
    msg.platform_state = "READY";
  } else {
    msg.platform_state = "DISABLED";
  }
  state_pub_->publish(msg);
}

}  // namespace puppypi_mock

// ===== 节点入口 =====
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<puppypi_mock::FakeMotionServerNode>());
  rclcpp::shutdown();
  return 0;
}
