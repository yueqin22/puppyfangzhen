// Watchdog & 故障恢复 (v7.10)
//
// 监控系统健康状态，在故障发生时执行恢复动作。
//
// 监控项:
//   1. 节点心跳超时检测
//   2. 传感器数据超时(LiDAR/IMU/Camera)
//   3. 通信超时(cmd_vel/odom)
//   4. 异常状态检测(跌倒/卡住/偏离)
//
// 恢复动作:
//   1. 重启节点
//   2. 清除故障状态
//   3. 触发姿态恢复
//   4. 紧急停止
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace puppy_core {

/// 看门狗状态（与Python WatchdogState IntEnum 一致）
enum class WatchdogState : int {
  HEALTHY = 0,         // 健康
  DEGRADED = 1,        // 降级
  FAULT = 2,           // 故障
  RECOVERING = 3,      // 恢复中
  EMERGENCY_STOP = 4,  // 紧急停止
};

/// 传感器健康状态
struct SensorHealth {
  std::string name;
  double last_update{0.0};   // 单位：秒（自节点启动起）
  double timeout{1.0};       // 超时阈值(秒)
  bool healthy{true};
  int failure_count{0};
};

/// Watchdog节点 (v7.10)
///
/// 监控系统健康，在故障时执行恢复。
class WatchdogNode : public rclcpp::Node {
 public:
  WatchdogNode();
  ~WatchdogNode() override = default;

 private:
  void onLidar(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg);
  void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg);

  void checkHealth();
  void triggerEmergency(const std::string& reason);
  void startRecovery();
  void checkRecovery();

  void recoverService(
      const std_srvs::srv::Trigger::Request::SharedPtr request,
      std_srvs::srv::Trigger::Response::SharedPtr response);
  void resetService(
      const std_srvs::srv::Trigger::Request::SharedPtr request,
      std_srvs::srv::Trigger::Response::SharedPtr response);

  // 将状态枚举转为字符串（用于发布与日志）
  static const char* stateName(WatchdogState s);

  // 当前秒数（使用 ROS 时钟，等价 Python time.time() 在节点时间轴上的语义）
  double nowSeconds() const;

 private:
  // 参数
  double check_interval_{0.5};
  double sensor_timeout_{1.0};
  double cmd_timeout_{0.5};
  int max_failures_{3};
  bool auto_recover_{true};

  // 状态
  WatchdogState state_{WatchdogState::HEALTHY};
  int failure_count_{0};
  int recovery_attempts_{0};

  // 传感器健康跟踪
  std::unordered_map<std::string, SensorHealth> sensors_;

  // 通信健康
  double last_cmd_vel_time_{0.0};

  // ROS2 接口
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr health_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr recover_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;

  rclcpp::TimerBase::SharedPtr check_timer_;
  // 单次恢复检查定时器（对应 Python 中 self.create_timer(3.0, ...) 的一次性回调）
  rclcpp::TimerBase::SharedPtr recovery_timer_;
};

}  // namespace puppy_core
