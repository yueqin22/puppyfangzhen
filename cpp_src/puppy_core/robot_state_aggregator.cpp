// Robot State Aggregator 实现
#include "puppy_core/robot_state_aggregator.h"

#include <algorithm>
#include <set>
#include <sstream>

namespace puppy_core {

// 默认超时阈值（秒）
static constexpr double kDefaultBatteryTimeout = 10.0;
static constexpr double kDefaultMotionTimeout = 5.0;
static constexpr double kDefaultImuTimeout = 2.0;
static constexpr double kDefaultLidarTimeout = 3.0;

RobotStateAggregatorNode::RobotStateAggregatorNode()
    : rclcpp::Node("robot_state_aggregator") {
  this->declare_parameter("battery_timeout", kDefaultBatteryTimeout);
  this->declare_parameter("motion_timeout", kDefaultMotionTimeout);
  this->declare_parameter("imu_timeout", kDefaultImuTimeout);
  this->declare_parameter("lidar_timeout", kDefaultLidarTimeout);
  battery_timeout_ = this->get_parameter("battery_timeout").as_double();
  motion_timeout_ = this->get_parameter("motion_timeout").as_double();
  imu_timeout_ = this->get_parameter("imu_timeout").as_double();
  lidar_timeout_ = this->get_parameter("lidar_timeout").as_double();

  battery_percent_ = 1.0;
  imu_ready_ = false;
  lidar_ready_ = false;
  camera_ready_ = false;
  motion_ready_ = false;
  has_platform_health_ = false;

  // 初始化数据源状态
  for (int i = 0; i < 4; ++i) {
    Source s = static_cast<Source>(i);
    has_seen_[s] = false;
    last_seen_[s] = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
  }

  health_pub_ = this->create_publisher<puppy_interfaces::msg::RobotHealth>(
      "/robot/health", 10);

  battery_sub_ =
      this->create_subscription<puppy_interfaces::msg::BatteryStatus>(
          "/battery_status", 10,
          std::bind(&RobotStateAggregatorNode::onBattery, this,
                    std::placeholders::_1));
  motion_sub_ =
      this->create_subscription<puppy_interfaces::msg::PlatformMotionState>(
          "/platform/motion_state", 10,
          std::bind(&RobotStateAggregatorNode::onMotion, this,
                    std::placeholders::_1));
  fall_sub_ = this->create_subscription<puppy_interfaces::msg::FallEvent>(
      "/fall/event", 10,
      std::bind(&RobotStateAggregatorNode::onFall, this,
                std::placeholders::_1));
  platform_health_sub_ =
      this->create_subscription<puppy_interfaces::msg::RobotHealth>(
          "/platform/health", 10,
          std::bind(&RobotStateAggregatorNode::onPlatformHealth, this,
                    std::placeholders::_1));
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", 10,
      std::bind(&RobotStateAggregatorNode::onImu, this,
                std::placeholders::_1));
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&RobotStateAggregatorNode::onScan, this,
                std::placeholders::_1));

  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&RobotStateAggregatorNode::publishHealth, this));

  RCLCPP_INFO(this->get_logger(),
              "Robot state aggregator started (timeouts: battery=%.1f, "
              "motion=%.1f, imu=%.1f, lidar=%.1f)",
              battery_timeout_, motion_timeout_, imu_timeout_, lidar_timeout_);
}

const char* RobotStateAggregatorNode::sourceName(Source s) {
  switch (s) {
    case Source::kBattery: return "battery";
    case Source::kMotion:  return "motion";
    case Source::kImu:     return "imu";
    case Source::kLidar:   return "lidar";
  }
  return "unknown";
}

std::string RobotStateAggregatorNode::sourceUpper(Source s) {
  std::string upper = sourceName(s);
  std::transform(upper.begin(), upper.end(), upper.begin(),
                 [](unsigned char c) { return std::toupper(c); });
  return upper;
}

void RobotStateAggregatorNode::onBattery(
    const puppy_interfaces::msg::BatteryStatus::SharedPtr msg) {
  battery_percent_ = std::max(0.0, std::min(1.0, static_cast<double>(msg->percent)));
  has_seen_[Source::kBattery] = true;
  last_seen_[Source::kBattery] = this->now();
}

void RobotStateAggregatorNode::onMotion(
    const puppy_interfaces::msg::PlatformMotionState::SharedPtr msg) {
  motion_ready_ = msg->controllable;
  has_seen_[Source::kMotion] = true;
  last_seen_[Source::kMotion] = this->now();
}

void RobotStateAggregatorNode::onImu(
    const sensor_msgs::msg::Imu::SharedPtr /*msg*/) {
  has_seen_[Source::kImu] = true;
  last_seen_[Source::kImu] = this->now();
}

void RobotStateAggregatorNode::onScan(
    const sensor_msgs::msg::LaserScan::SharedPtr /*msg*/) {
  has_seen_[Source::kLidar] = true;
  last_seen_[Source::kLidar] = this->now();
}

void RobotStateAggregatorNode::onFall(
    const puppy_interfaces::msg::FallEvent::SharedPtr msg) {
  const std::string fault = "FALL_DETECTED";
  if (msg->detected) {
    if (std::find(active_faults_.begin(), active_faults_.end(), fault) ==
        active_faults_.end()) {
      active_faults_.push_back(fault);
    }
  } else {
    auto it = std::find(active_faults_.begin(), active_faults_.end(), fault);
    if (it != active_faults_.end()) {
      active_faults_.erase(it);
    }
  }
}

void RobotStateAggregatorNode::onPlatformHealth(
    const puppy_interfaces::msg::RobotHealth::SharedPtr msg) {
  has_platform_health_ = true;
  platform_health_ = *msg;
  camera_ready_ = msg->camera_ready;
}

std::vector<std::string> RobotStateAggregatorNode::checkTimeouts() {
  rclcpp::Time now = this->now();
  std::vector<std::string> timeout_faults;

  const Source sources[] = {Source::kBattery, Source::kMotion,
                            Source::kImu, Source::kLidar};
  for (Source s : sources) {
    if (!has_seen_[s]) {
      timeout_faults.push_back(sourceUpper(s) + "_OFFLINE");
      continue;
    }
    double elapsed = (now - last_seen_[s]).seconds();
    double threshold = 0.0;
    switch (s) {
      case Source::kBattery: threshold = battery_timeout_; break;
      case Source::kMotion:  threshold = motion_timeout_;  break;
      case Source::kImu:     threshold = imu_timeout_;     break;
      case Source::kLidar:   threshold = lidar_timeout_;   break;
    }
    if (elapsed > threshold) {
      timeout_faults.push_back(sourceUpper(s) + "_TIMEOUT");
    }
  }

  // 新增的故障告警
  for (const auto& fault : timeout_faults) {
    if (reported_timeout_faults_.find(fault) == reported_timeout_faults_.end()) {
      RCLCPP_WARN(this->get_logger(), "Sensor timeout: %s", fault.c_str());
    }
  }

  // 已恢复的故障
  std::set<std::string> current_set(timeout_faults.begin(),
                                    timeout_faults.end());
  for (const auto& fault : reported_timeout_faults_) {
    if (current_set.find(fault) == current_set.end()) {
      RCLCPP_INFO(this->get_logger(), "Sensor recovered: %s", fault.c_str());
    }
  }
  reported_timeout_faults_ = current_set;

  return timeout_faults;
}

void RobotStateAggregatorNode::publishHealth() {
  puppy_interfaces::msg::RobotHealth msg;
  msg.header.stamp = this->now();

  std::vector<std::string> timeout_faults = checkTimeouts();

  // 提取超时故障的源前缀（如 "BATTERY_TIMEOUT" -> "BATTERY"）
  std::set<std::string> timeout_set;
  for (const auto& fault : timeout_faults) {
    auto underscore = fault.find('_');
    if (underscore != std::string::npos) {
      timeout_set.insert(fault.substr(0, underscore));
    } else {
      timeout_set.insert(fault);
    }
  }

  imu_ready_ = (timeout_set.find("IMU") == timeout_set.end()) &&
               has_seen_[Source::kImu];
  lidar_ready_ = (timeout_set.find("LIDAR") == timeout_set.end()) &&
                 has_seen_[Source::kLidar];
  bool battery_ok = (timeout_set.find("BATTERY") == timeout_set.end());
  if (timeout_set.find("MOTION") != timeout_set.end()) {
    motion_ready_ = false;
  }

  std::vector<std::string> active_faults = active_faults_;
  if (has_platform_health_) {
    for (const auto& fault : platform_health_.active_faults) {
      if (std::find(active_faults.begin(), active_faults.end(), fault) ==
          active_faults.end()) {
        active_faults.push_back(fault);
      }
    }
  }
  for (const auto& fault : timeout_faults) {
    if (std::find(active_faults.begin(), active_faults.end(), fault) ==
        active_faults.end()) {
      active_faults.push_back(fault);
    }
  }

  msg.cpu_temp =
      has_platform_health_ ? platform_health_.cpu_temp : 0.0f;
  msg.ok = active_faults.empty() && battery_ok && battery_percent_ > 0.1;
  msg.level = msg.ok ? "OK" : "ERROR";
  msg.active_faults = active_faults;
  msg.battery_percent =
      battery_ok ? static_cast<float>(battery_percent_ * 100.0) : 0.0f;
  msg.imu_ready = imu_ready_;
  msg.lidar_ready = lidar_ready_;
  msg.camera_ready = camera_ready_;
  msg.motion_ready = motion_ready_;
  health_pub_->publish(msg);
}

}  // namespace puppy_core
