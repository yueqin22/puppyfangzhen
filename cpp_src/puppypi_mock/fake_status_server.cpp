// Fake Status Server 实现
//
// 仿真 PuppyPi 电池、关节、健康状态
#include "puppypi_mock/fake_status_server.h"

#include <algorithm>
#include <chrono>
#include <string>
#include <vector>

namespace puppypi_mock {

FakeStatusServerNode::FakeStatusServerNode()
    : rclcpp::Node("fake_status_server") {
  // 电池初始状态
  battery_percent_ = 0.85;
  battery_drain_rate_ = 0.0001;  // 每个周期消耗

  // 发布器
  battery_pub_ =
      this->create_publisher<sensor_msgs::msg::BatteryState>("/battery_state",
                                                             10);
  joint_pub_ =
      this->create_publisher<sensor_msgs::msg::JointState>("/joint_states",
                                                           10);
  health_pub_ =
      this->create_publisher<puppy_interfaces::msg::RobotHealth>(
          "/platform/health", 10);

  // 状态定时器（20Hz）
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&FakeStatusServerNode::publishStatus, this));

  RCLCPP_INFO(this->get_logger(), "Fake status server started (battery=85%%)");
}

void FakeStatusServerNode::publishStatus() {
  // 模拟电池消耗
  battery_percent_ =
      std::max(0.0, battery_percent_ - battery_drain_rate_);

  builtin_interfaces::msg::Time now_stamp = this->now();

  // ===== 电池状态 =====
  sensor_msgs::msg::BatteryState bs;
  bs.header.stamp = now_stamp;
  bs.voltage = 12.0 * (0.8 + 0.2 * battery_percent_);
  bs.current = -1.0;
  bs.percentage = battery_percent_;
  bs.power_supply_status =
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
  battery_pub_->publish(bs);

  // ===== 关节状态（8DOF: 4腿 x 2关节）=====
  sensor_msgs::msg::JointState js;
  js.header.stamp = now_stamp;
  // 关节名：leg0_hip, leg0_knee, leg1_hip, leg1_knee, ...
  js.name.reserve(8);
  for (int i = 0; i < 4; ++i) {
    js.name.push_back("leg" + std::to_string(i) + "_hip");
    js.name.push_back("leg" + std::to_string(i) + "_knee");
  }
  js.position = std::vector<double>(8, 0.0);
  joint_pub_->publish(js);

  // ===== 健康状态 =====
  puppy_interfaces::msg::RobotHealth h;
  h.header.stamp = now_stamp;
  h.ok = battery_percent_ > 0.1;
  h.level = h.ok ? "OK" : "ERROR";
  h.battery_percent = battery_percent_ * 100.0;
  h.imu_ready = true;
  h.lidar_ready = true;
  h.camera_ready = true;
  h.motion_ready = true;
  health_pub_->publish(h);
}

}  // namespace puppypi_mock

// ===== 节点入口 =====
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<puppypi_mock::FakeStatusServerNode>());
  rclcpp::shutdown();
  return 0;
}
