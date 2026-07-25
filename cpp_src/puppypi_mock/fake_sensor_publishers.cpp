// Fake Sensor Publishers 实现
//
// 仿真 IMU、LiDAR、相机数据用于测试
#include "puppypi_mock/fake_sensor_publishers.h"

#include <chrono>
#include <cmath>
#include <random>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace puppypi_mock {

FakeSensorPublishersNode::FakeSensorPublishersNode()
    : rclcpp::Node("fake_sensors") {
  // 发布器
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
  scan_pub_ =
      this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
  camera_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
      "/camera/color/image_raw", 10);

  // 传感器定时器（按真实频率）
  imu_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),  // 100Hz
      std::bind(&FakeSensorPublishersNode::publishImu, this));
  scan_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),  // 20Hz
      std::bind(&FakeSensorPublishersNode::publishScan, this));
  camera_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(33),  // ~30Hz
      std::bind(&FakeSensorPublishersNode::publishCamera, this));

  RCLCPP_INFO(this->get_logger(),
              "Fake sensors started (IMU=100Hz, LiDAR=20Hz, Camera=30Hz)");
}

void FakeSensorPublishersNode::publishImu() {
  sensor_msgs::msg::Imu msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "imu_link";
  msg.orientation.w = 1.0;
  // 协方差矩阵（3x3，共9个元素）
  msg.orientation_covariance = {0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01};
  msg.angular_velocity_covariance = {0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01};
  msg.linear_acceleration_covariance = {0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1};
  imu_pub_->publish(msg);
}

void FakeSensorPublishersNode::publishScan() {
  sensor_msgs::msg::LaserScan msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "laser_link";
  msg.angle_min = -M_PI;
  msg.angle_max = M_PI;
  msg.angle_increment = 2.0 * M_PI / 72.0;
  msg.time_increment = 0.001;
  msg.scan_time = 0.05;
  msg.range_min = 0.1;
  msg.range_max = 8.0;

  // 仿真 72 条射线，随机距离
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> dist(1.0, 5.0);
  msg.ranges.resize(72);
  for (int i = 0; i < 72; ++i) {
    msg.ranges[i] = dist(gen);
  }
  scan_pub_->publish(msg);
}

void FakeSensorPublishersNode::publishCamera() {
  sensor_msgs::msg::Image msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "camera_color_optical_frame";
  msg.height = 240;
  msg.width = 320;
  msg.encoding = "rgb8";
  msg.is_bigendian = false;
  msg.step = 320 * 3;
  // 灰色图像（所有像素 = 128）
  msg.data.assign(240 * 320 * 3, 128);
  camera_pub_->publish(msg);
}

}  // namespace puppypi_mock

// ===== 节点入口 =====
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<puppypi_mock::FakeSensorPublishersNode>());
  rclcpp::shutdown();
  return 0;
}
