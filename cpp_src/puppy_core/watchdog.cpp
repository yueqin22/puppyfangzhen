// Watchdog & 故障恢复 实现 (v7.10)
#include "puppy_core/watchdog.h"

#include <cmath>
#include <utility>

namespace puppy_core {

WatchdogNode::WatchdogNode() : rclcpp::Node("watchdog") {
  // 参数
  check_interval_ = this->declare_parameter("check_interval", 0.5);
  sensor_timeout_ = this->declare_parameter("sensor_timeout", 1.0);
  cmd_timeout_ = this->declare_parameter("cmd_timeout", 0.5);
  max_failures_ = this->declare_parameter("max_failures", 3);
  auto_recover_ = this->declare_parameter("auto_recover", true);

  // 传感器健康跟踪
  SensorHealth lidar;
  lidar.name = "lidar";
  lidar.timeout = sensor_timeout_;
  sensors_["lidar"] = lidar;

  SensorHealth imu;
  imu.name = "imu";
  imu.timeout = sensor_timeout_;
  sensors_["imu"] = imu;

  // 通信健康
  last_cmd_vel_time_ = nowSeconds();

  // ROS2接口
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&WatchdogNode::onLidar, this, std::placeholders::_1));
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", 10,
      std::bind(&WatchdogNode::onImu, this, std::placeholders::_1));
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&WatchdogNode::onCmdVel, this, std::placeholders::_1));

  health_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/watchdog/health", 10);
  emergency_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/watchdog/emergency_stop", 10);
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10);

  // 服务
  recover_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/watchdog/recover",
      std::bind(&WatchdogNode::recoverService, this,
                std::placeholders::_1, std::placeholders::_2));
  reset_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/watchdog/reset",
      std::bind(&WatchdogNode::resetService, this,
                std::placeholders::_1, std::placeholders::_2));

  // 检查定时器
  check_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(check_interval_ * 1000)),
      std::bind(&WatchdogNode::checkHealth, this));
  RCLCPP_INFO(this->get_logger(), "Watchdog节点已启动 (v7.10)");
}

double WatchdogNode::nowSeconds() const {
  // 使用 ROS 时钟，等价 Python time.time() 相对于节点启动的语义
  return this->now().seconds();
}

const char* WatchdogNode::stateName(WatchdogState s) {
  switch (s) {
    case WatchdogState::HEALTHY:        return "HEALTHY";
    case WatchdogState::DEGRADED:       return "DEGRADED";
    case WatchdogState::FAULT:          return "FAULT";
    case WatchdogState::RECOVERING:     return "RECOVERING";
    case WatchdogState::EMERGENCY_STOP: return "EMERGENCY_STOP";
  }
  return "UNKNOWN";
}

void WatchdogNode::onLidar(const sensor_msgs::msg::LaserScan::SharedPtr /*msg*/) {
  // LiDAR数据更新
  sensors_["lidar"].last_update = nowSeconds();
  sensors_["lidar"].healthy = true;
}

void WatchdogNode::onImu(const sensor_msgs::msg::Imu::SharedPtr msg) {
  // IMU数据更新
  sensors_["imu"].last_update = nowSeconds();
  sensors_["imu"].healthy = true;
  // 检测跌倒（加速度异常）
  const auto& accel = msg->linear_acceleration;
  double accel_mag = std::sqrt(accel.x * accel.x +
                               accel.y * accel.y +
                               accel.z * accel.z);
  if (accel_mag > 25.0 || accel_mag < 5.0) {  // 正常约9.8
    RCLCPP_ERROR(this->get_logger(),
                 "检测到异常加速度: %.1f m/s² — 可能跌倒", accel_mag);
    triggerEmergency("fall_detected");
  }
}

void WatchdogNode::onCmdVel(const geometry_msgs::msg::Twist::SharedPtr /*msg*/) {
  // cmd_vel更新
  last_cmd_vel_time_ = nowSeconds();
}

void WatchdogNode::checkHealth() {
  // 定期健康检查
  double now = nowSeconds();
  bool any_unhealthy = false;
  bool critical_failure = false;

  for (auto& kv : sensors_) {
    SensorHealth& sensor = kv.second;
    if (sensor.last_update > 0.0) {
      double elapsed = now - sensor.last_update;
      if (elapsed > sensor.timeout) {
        sensor.healthy = false;
        sensor.failure_count += 1;
        any_unhealthy = true;
        if (sensor.failure_count > max_failures_) {
          critical_failure = true;
        }
        RCLCPP_WARN(this->get_logger(),
                    "传感器 %s 超时 (%.1fs)", kv.first.c_str(), elapsed);
      } else {
        sensor.healthy = true;
        sensor.failure_count = 0;
      }
    }
  }

  // 更新状态
  if (critical_failure) {
    if (state_ != WatchdogState::EMERGENCY_STOP) {
      triggerEmergency("critical_sensor_failure");
    }
  } else if (any_unhealthy) {
    if (state_ == WatchdogState::HEALTHY) {
      state_ = WatchdogState::DEGRADED;
      RCLCPP_WARN(this->get_logger(), "系统降级运行");
    }
  } else {
    if (state_ == WatchdogState::DEGRADED) {
      state_ = WatchdogState::HEALTHY;
      RCLCPP_INFO(this->get_logger(), "系统恢复正常");
    }
  }

  // 发布健康状态
  std_msgs::msg::String health_msg;
  health_msg.data = stateName(state_);
  health_pub_->publish(health_msg);
}

void WatchdogNode::triggerEmergency(const std::string& reason) {
  // 触发紧急停止
  state_ = WatchdogState::EMERGENCY_STOP;
  // 紧急停止
  geometry_msgs::msg::Twist cmd;
  cmd_vel_pub_->publish(cmd);
  // 发布紧急停止信号
  std_msgs::msg::Bool emergency_msg;
  emergency_msg.data = true;
  emergency_pub_->publish(emergency_msg);
  RCLCPP_ERROR(this->get_logger(), "紧急停止: %s", reason.c_str());

  // 尝试自动恢复
  if (auto_recover_) {
    startRecovery();
  }
}

void WatchdogNode::startRecovery() {
  // 启动故障恢复流程
  state_ = WatchdogState::RECOVERING;
  recovery_attempts_ += 1;
  RCLCPP_INFO(this->get_logger(), "启动恢复流程 (第%d次)", recovery_attempts_);

  // 恢复步骤：
  // 1. 清除紧急停止
  std_msgs::msg::Bool emergency_msg;
  emergency_msg.data = false;
  emergency_pub_->publish(emergency_msg);

  // 2. 重置传感器计数
  for (auto& kv : sensors_) {
    kv.second.failure_count = 0;
  }

  // 3. 检查是否恢复（3秒后单次回调）
  recovery_timer_ = this->create_wall_timer(
      std::chrono::seconds(3),
      [this]() {
        // 单次回调：取消定时器后执行检查
        recovery_timer_->cancel();
        checkRecovery();
      });
}

void WatchdogNode::checkRecovery() {
  // 检查恢复结果（单次定时回调）
  bool all_healthy = true;
  for (const auto& kv : sensors_) {
    if (!kv.second.healthy) {
      all_healthy = false;
      break;
    }
  }
  if (all_healthy) {
    state_ = WatchdogState::HEALTHY;
    RCLCPP_INFO(this->get_logger(), "恢复成功");
  } else {
    state_ = WatchdogState::FAULT;
    RCLCPP_ERROR(this->get_logger(), "恢复失败 — 需要人工干预");
  }
}

void WatchdogNode::recoverService(
    const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
    std_srvs::srv::Trigger::Response::SharedPtr response) {
  // 手动恢复服务
  startRecovery();
  response->success = true;
  response->message = "恢复流程已启动";
}

void WatchdogNode::resetService(
    const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
    std_srvs::srv::Trigger::Response::SharedPtr response) {
  // 重置看门狗
  state_ = WatchdogState::HEALTHY;
  failure_count_ = 0;
  recovery_attempts_ = 0;
  for (auto& kv : sensors_) {
    kv.second.failure_count = 0;
    kv.second.healthy = true;
  }
  // 清除紧急停止
  std_msgs::msg::Bool emergency_msg;
  emergency_msg.data = false;
  emergency_pub_->publish(emergency_msg);
  response->success = true;
  response->message = "看门狗已重置";
  RCLCPP_INFO(this->get_logger(), "看门狗已重置");
}

}  // namespace puppy_core
