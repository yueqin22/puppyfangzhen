// 电池仿真节点实现
#include "puppy_gait/battery_simulator.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace puppy_gait {

BatterySimulatorNode::BatterySimulatorNode()
    : rclcpp::Node("battery_simulator") {
  this->declare_parameter("publish_semantic_battery", true);
  publish_semantic_battery_ =
      this->get_parameter("publish_semantic_battery").as_bool();

  // ===== 仿真电池参数（直接使用每秒百分比，方便测试）=====
  // drain_rate_idle_ = 0.001  (空闲: 0.1%/秒)
  // drain_rate_active_ = 0.003 (移动: 0.3%/秒)
  // charge_rate_ = 0.01        (充电: 1.0%/秒)

  // ===== 充电桩位置（从 navigation_targets.yaml 加载，默认值）=====
  // 对应 shared_targets.get_named_target('dock', {'x': 0.0, 'y': -2.0, 'yaw': 0.0})
  dock_x_ = 0.0;
  dock_y_ = -2.0;

  // ===== 状态 =====
  battery_level_ = 1.0;
  is_charging_ = false;
  is_moving_ = false;
  robot_x_ = 1.0;
  robot_y_ = -2.0;
  low_battery_alerted_ = false;
  last_logged_percent_ = -1;

  // ===== 发布器 =====
  // P0-1: 发布语义消息 puppy_interfaces/BatteryStatus 到 /battery_status
  // 供 puppy_core（mode_manager/safety_manager/mission_manager）订阅
  // 同时保留 sensor_msgs/BatteryState 到 /battery_state 作为原始兼容 topic
  battery_pub_ =
      this->create_publisher<sensor_msgs::msg::BatteryState>("/battery_state", 10);
  battery_semantic_pub_ =
      this->create_publisher<puppy_interfaces::msg::BatteryStatus>(
          "/battery_status", 10);
  low_battery_pub_ =
      this->create_publisher<std_msgs::msg::Bool>("/low_battery_alert", 10);

  // ===== 订阅器 =====
  pose_sub_ =
      this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/amcl_pose", 10,
          std::bind(&BatterySimulatorNode::poseCallback, this,
                    std::placeholders::_1));

  // ===== 定时器（5Hz 更新电池状态）=====
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(200),
      std::bind(&BatterySimulatorNode::updateBattery, this));
  last_time_ = this->now();

  RCLCPP_INFO(this->get_logger(), "%s", "==================================================");
  RCLCPP_INFO(this->get_logger(), "  电池仿真节点启动（仿真时间模式）");
  RCLCPP_INFO(this->get_logger(), "  空闲耗电: %.2f%%/秒", drain_rate_idle_ * 100.0);
  RCLCPP_INFO(this->get_logger(), "  移动耗电: %.2f%%/秒", drain_rate_active_ * 100.0);
  RCLCPP_INFO(this->get_logger(), "  充电速率: %.2f%%/秒", charge_rate_ * 100.0);
  RCLCPP_INFO(this->get_logger(), "  充电桩位置: (%.1f, %.1f)", dock_x_, dock_y_);
  RCLCPP_INFO(this->get_logger(), "  低电量阈值: %.0f%%",
              low_battery_threshold_ * 100.0);
  RCLCPP_INFO(this->get_logger(), "%s", "==================================================");
}

void BatterySimulatorNode::poseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  if (last_position_.has_value()) {
    double dx = robot_x_ - last_position_->first;
    double dy = robot_y_ - last_position_->second;
    double distance = std::sqrt(dx * dx + dy * dy);
    is_moving_ = distance > 0.005;
  }
  last_position_ = std::make_pair(robot_x_, robot_y_);
}

void BatterySimulatorNode::updateBattery() {
  rclcpp::Time now = this->now();
  double dt = (now - last_time_).seconds();
  last_time_ = now;

  if (dt <= 0 || dt > 1.0) {
    return;
  }

  double dist_to_dock = std::sqrt(
      (robot_x_ - dock_x_) * (robot_x_ - dock_x_) +
      (robot_y_ - dock_y_) * (robot_y_ - dock_y_));

  if (dist_to_dock < dock_range_) {
    is_charging_ = true;
    battery_level_ += charge_rate_ * dt;
    battery_level_ = std::min(1.0, battery_level_);
  } else {
    is_charging_ = false;
    double drain = is_moving_ ? drain_rate_active_ : drain_rate_idle_;
    battery_level_ -= drain * dt;
    battery_level_ = std::max(0.0, battery_level_);
  }

  publishBatteryState();

  if (battery_level_ <= low_battery_threshold_) {
    if (!low_battery_alerted_) {
      RCLCPP_WARN(this->get_logger(), "低电量警报！当前电量: %.1f%%",
                  battery_level_ * 100.0);
      std_msgs::msg::Bool alert;
      alert.data = true;
      low_battery_pub_->publish(alert);
      low_battery_alerted_ = true;
    }
  } else {
    if (low_battery_alerted_ && battery_level_ > 0.3) {
      RCLCPP_INFO(this->get_logger(), "电量恢复正常: %.1f%%",
                  battery_level_ * 100.0);
      std_msgs::msg::Bool alert;
      alert.data = false;
      low_battery_pub_->publish(alert);
      low_battery_alerted_ = false;
    }
  }

  int current_percent = static_cast<int>(battery_level_ * 100) / 10 * 10;
  if (is_charging_ && battery_level_ < 1.0) {
    if (current_percent != last_logged_percent_) {
      last_logged_percent_ = current_percent;
      RCLCPP_INFO(this->get_logger(), "充电中... 电量: %.1f%%",
                  battery_level_ * 100.0);
    }
  } else if (!is_charging_ && battery_level_ >= 1.0) {
    last_logged_percent_ = 100;
  }
}

void BatterySimulatorNode::publishBatteryState() {
  sensor_msgs::msg::BatteryState msg;
  msg.header.stamp = this->now();
  msg.header.frame_id = "base_link";
  msg.voltage = voltage_empty_ +
                (voltage_full_ - voltage_empty_) * battery_level_;
  if (is_charging_) {
    msg.current = 1.0;
  } else {
    msg.current = -(is_moving_ ? 1.0 : 0.3);
  }
  msg.percentage = battery_level_;
  if (battery_level_ >= 1.0 && is_charging_) {
    msg.power_supply_status =
        sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL;
  } else if (is_charging_) {
    msg.power_supply_status =
        sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
  } else {
    msg.power_supply_status =
        sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
  }
  msg.power_supply_health =
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
  msg.power_supply_technology =
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
  msg.present = true;
  battery_pub_->publish(msg);

  // P0-1: 发布语义消息 BatteryStatus 到 /battery_status
  // 供 puppy_core 订阅（mode_manager/safety_manager/mission_manager）
  puppy_interfaces::msg::BatteryStatus sem;
  sem.header = msg.header;
  sem.voltage = msg.voltage;
  sem.current = msg.current;
  sem.percent = battery_level_;
  sem.charging = is_charging_;
  sem.low_battery = battery_level_ <= low_battery_threshold_;
  sem.critical_battery = battery_level_ <= 0.10;
  if (publish_semantic_battery_) {
    battery_semantic_pub_->publish(sem);
  }
}

}  // namespace puppy_gait
