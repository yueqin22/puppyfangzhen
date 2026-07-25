// Robot State Aggregator: unified robot health from semantic topics.
//
// 将来自多个语义话题的状态聚合为统一的 RobotHealth 消息。
#pragma once

#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "puppy_interfaces/msg/battery_status.hpp"
#include "puppy_interfaces/msg/fall_event.hpp"
#include "puppy_interfaces/msg/platform_motion_state.hpp"
#include "puppy_interfaces/msg/robot_health.hpp"

namespace puppy_core {

/// Aggregates robot state from multiple sources into unified health.
class RobotStateAggregatorNode : public rclcpp::Node {
 public:
  RobotStateAggregatorNode();
  ~RobotStateAggregatorNode() override = default;

 private:
  // 数据源标识
  enum class Source { kBattery, kMotion, kImu, kLidar };

  void onBattery(const puppy_interfaces::msg::BatteryStatus::SharedPtr msg);
  void onMotion(
      const puppy_interfaces::msg::PlatformMotionState::SharedPtr msg);
  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg);
  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void onFall(const puppy_interfaces::msg::FallEvent::SharedPtr msg);
  void onPlatformHealth(
      const puppy_interfaces::msg::RobotHealth::SharedPtr msg);

  // 检查超时，返回当前超时故障列表
  std::vector<std::string> checkTimeouts();

  void publishHealth();

  // 将 Source 转为字符串
  static const char* sourceName(Source s);
  // 将 Source 转为大写前缀（用于故障字符串，如 BATTERY_TIMEOUT）
  static std::string sourceUpper(Source s);

 private:
  // 超时阈值（秒）
  double battery_timeout_{10.0};
  double motion_timeout_{5.0};
  double imu_timeout_{2.0};
  double lidar_timeout_{3.0};

  double battery_percent_{1.0};
  bool imu_ready_{false};
  bool lidar_ready_{false};
  bool camera_ready_{false};
  bool motion_ready_{false};
  std::vector<std::string> active_faults_;
  bool has_platform_health_{false};
  puppy_interfaces::msg::RobotHealth platform_health_;

  // 最近一次收到各数据源的时间（None 用 has_seen 标志表示）
  std::unordered_map<Source, rclcpp::Time> last_seen_;
  std::unordered_map<Source, bool> has_seen_;

  std::set<std::string> reported_timeout_faults_;

  rclcpp::Publisher<puppy_interfaces::msg::RobotHealth>::SharedPtr health_pub_;

  rclcpp::Subscription<puppy_interfaces::msg::BatteryStatus>::SharedPtr
      battery_sub_;
  rclcpp::Subscription<puppy_interfaces::msg::PlatformMotionState>::SharedPtr
      motion_sub_;
  rclcpp::Subscription<puppy_interfaces::msg::FallEvent>::SharedPtr fall_sub_;
  rclcpp::Subscription<puppy_interfaces::msg::RobotHealth>::SharedPtr
      platform_health_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace puppy_core
