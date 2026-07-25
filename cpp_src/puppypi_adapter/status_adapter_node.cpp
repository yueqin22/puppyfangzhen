// Status Adapter Node 实现
//
// 将 PuppyPi 平台状态转换为标准 ROS2 消息
#include "puppypi_adapter/status_adapter_node.h"

#include <algorithm>
#include <stdexcept>
#include <string>

namespace puppypi_adapter {

StatusAdapterNode::StatusAdapterNode()
    : rclcpp::Node("status_adapter") {
  // 参数声明
  this->declare_parameter("use_sim", true);
  this->declare_parameter("publish_raw_battery", true);
  this->declare_parameter("publish_semantic_battery", true);
  this->declare_parameter("battery_drain_per_tick", 0.0002);

  use_sim_ = this->get_parameter("use_sim").as_bool();
  publish_raw_battery_ = this->get_parameter("publish_raw_battery").as_bool();
  publish_semantic_battery_ =
      this->get_parameter("publish_semantic_battery").as_bool();
  battery_drain_per_tick_ =
      this->get_parameter("battery_drain_per_tick").as_double();

  battery_percent_ = 0.80;
  sdk_connected_ = false;

  // 关节名称：leg0_hip, leg0_knee, ..., leg3_knee（共 8 个）
  const std::string joint_suffixes[2] = {"hip", "knee"};
  joint_names_.reserve(8);
  for (int i = 0; i < 4; ++i) {
    for (const auto& j : joint_suffixes) {
      joint_names_.push_back("leg" + std::to_string(i) + "_" + j);
    }
  }

  // 发布器
  health_pub_ = this->create_publisher<puppy_interfaces::msg::RobotHealth>(
      "/platform/health", 10);
  joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "/joint_states", 10);
  battery_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>(
      "/battery_state", 10);
  battery_semantic_pub_ =
      this->create_publisher<puppy_interfaces::msg::BatteryStatus>(
          "/battery_status", 10);

  // 定时器（20Hz 轮询）
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&StatusAdapterNode::pollStatus, this));

  // 硬件模式：PuppyPi SDK 未集成
  if (!use_sim_) {
    RCLCPP_FATAL(this->get_logger(),
                 "use_sim=False 但 PuppyPi SDK 不可用。"
                 "请在仿真模式下设置 use_sim:=true。");
    throw std::runtime_error("PuppyPi SDK not yet integrated");
  }

  const char* mode_tag = use_sim_ ? "[SIM]" : "[HARDWARE]";
  RCLCPP_INFO(this->get_logger(), "Status adapter started %s", mode_tag);
}

void StatusAdapterNode::pollStatus() {
  // 轮询 PuppyPi 状态并发布到 ROS2
  battery_percent_ = std::max(
      0.0, battery_percent_ - battery_drain_per_tick_);

  builtin_interfaces::msg::Time now_stamp = this->now();

  // ===== /joint_states =====
  sensor_msgs::msg::JointState js;
  js.header.stamp = now_stamp;
  js.name = joint_names_;
  js.position = std::vector<double>(8, 0.0);
  js.velocity = std::vector<double>(8, 0.0);
  js.effort = std::vector<double>(8, 0.0);
  joint_pub_->publish(js);

  // ===== /battery_state (原始) =====
  sensor_msgs::msg::BatteryState bs;
  bs.header.stamp = now_stamp;
  bs.voltage = 12.0 * (0.8 + 0.2 * battery_percent_);
  bs.current = -0.8;
  bs.percentage = battery_percent_;
  bs.power_supply_status =
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
  if (publish_raw_battery_) {
    battery_pub_->publish(bs);
  }

  // ===== /battery_status (语义) =====
  if (publish_semantic_battery_) {
    puppy_interfaces::msg::BatteryStatus sem;
    sem.header = bs.header;
    sem.voltage = bs.voltage;
    sem.current = bs.current;
    sem.percent = battery_percent_;
    sem.charging = false;
    sem.low_battery = battery_percent_ <= 0.20;
    sem.critical_battery = battery_percent_ <= 0.10;
    battery_semantic_pub_->publish(sem);
  }

  // ===== /platform/health =====
  puppy_interfaces::msg::RobotHealth health;
  health.header.stamp = bs.header.stamp;
  health.ok = battery_percent_ > 0.10;
  if (!sdk_connected_) {
    health.level = "WARN";
  } else {
    health.level = health.ok ? "OK" : "ERROR";
  }
  // active_faults
  if (!sdk_connected_) {
    health.active_faults.push_back("STATUS_SOURCE_SIMULATED");
  }
  if (battery_percent_ <= 0.10) {
    health.active_faults.push_back("BATTERY_CRITICAL");
  } else if (battery_percent_ <= 0.20) {
    health.active_faults.push_back("BATTERY_LOW");
  }
  health.cpu_temp = 0.0;
  health.battery_percent = battery_percent_ * 100.0;
  health.imu_ready = sdk_connected_;
  health.lidar_ready = sdk_connected_;
  health.camera_ready = sdk_connected_;
  health.motion_ready = true;
  health_pub_->publish(health);
}

}  // namespace puppypi_adapter
