// Fake Sensor Publishers: 仿真 IMU、LiDAR、相机数据用于测试 (C++ 版本)
//
// 对应 Python: puppypi_mock/fake_sensor_publishers.py
#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace puppypi_mock {

/// 仿真传感器数据节点（用于无硬件开发）
class FakeSensorPublishersNode : public rclcpp::Node {
 public:
  FakeSensorPublishersNode();
  ~FakeSensorPublishersNode() override = default;

 private:
  // 发布 IMU 数据（100Hz）
  void publishImu();
  // 发布 LaserScan 数据（20Hz）
  void publishScan();
  // 发布相机图像（30Hz）
  void publishCamera();

 private:
  // ===== 发布器 =====
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_pub_;

  // ===== 定时器 =====
  rclcpp::TimerBase::SharedPtr imu_timer_;    // 100Hz
  rclcpp::TimerBase::SharedPtr scan_timer_;   // 20Hz
  rclcpp::TimerBase::SharedPtr camera_timer_;  // 30Hz
};

}  // namespace puppypi_mock
