// Dock感知闭环 实现 (v7.9)
//
// 检测充电桩并执行自动回充流程
#include "puppy_core/dock_detector.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace puppy_core {

DockDetectorNode::DockDetectorNode() : rclcpp::Node("dock_detector") {
  // 参数声明
  this->declare_parameter("dock_x", 0.0);
  this->declare_parameter("dock_y", -3.5);
  this->declare_parameter("dock_yaw", 0.0);
  this->declare_parameter("approach_speed", 0.1);
  this->declare_parameter("align_threshold", 0.1);         // 对齐阈值(弧度)
  this->declare_parameter("dock_distance_threshold", 0.3); // 对接距离
  this->declare_parameter("search_timeout", 30.0);         // 搜索超时(秒)

  // dock目标位置
  dock_x_ = this->get_parameter("dock_x").as_double();
  dock_y_ = this->get_parameter("dock_y").as_double();
  dock_yaw_ = this->get_parameter("dock_yaw").as_double();
  approach_speed_ = this->get_parameter("approach_speed").as_double();
  align_threshold_ = this->get_parameter("align_threshold").as_double();
  dock_distance_threshold_ =
      this->get_parameter("dock_distance_threshold").as_double();
  search_timeout_ = this->get_parameter("search_timeout").as_double();

  // 状态初始化
  state_ = DockState::IDLE;
  detection_ = DockDetection();
  state_start_time_ = this->now();

  // ROS2接口
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&DockDetectorNode::onScan, this, std::placeholders::_1));
  cmd_vel_pub_ =
      this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  state_pub_ =
      this->create_publisher<std_msgs::msg::String>("/dock/state", 10);
  start_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/dock/start_docking",
      std::bind(&DockDetectorNode::startDocking, this, std::placeholders::_1,
                std::placeholders::_2));
  cancel_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/dock/cancel",
      std::bind(&DockDetectorNode::cancelDocking, this, std::placeholders::_1,
                std::placeholders::_2));

  // 控制循环（10Hz）
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&DockDetectorNode::controlLoop, this));

  RCLCPP_INFO(this->get_logger(),
              "Dock感知节点已启动 (v7.9), dock位置: (%.1f, %.1f)",
              dock_x_, dock_y_);
}

void DockDetectorNode::onScan(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  // 处理LiDAR扫描，检测dock特征
  // dock特征：特定距离和角度范围内的反射点

  // 等价 Python:
  //   angles = np.arange(len(msg.ranges)) * msg.angle_increment + msg.angle_min
  //   distances = np.array(msg.ranges)
  //   front_mask = (np.abs(angles) < 0.5) & (distances < 1.0) & (distances > 0.1)
  //   if np.any(front_mask): ...

  const size_t n = msg->ranges.size();
  if (n == 0) {
    detection_.detected = false;
    return;
  }

  // 简化检测：寻找前方近距离的特征点
  // 收集满足 front_mask 条件的点
  std::vector<double> front_dists;
  std::vector<double> front_angles;
  front_dists.reserve(n);
  front_angles.reserve(n);
  for (size_t i = 0; i < n; ++i) {
    double angle = static_cast<double>(i) * msg->angle_increment + msg->angle_min;
    double dist = msg->ranges[i];
    if (std::abs(angle) < 0.5 && dist < 1.0 && dist > 0.1) {
      front_dists.push_back(dist);
      front_angles.push_back(angle);
    }
  }

  if (!front_dists.empty()) {
    // 检测dock的V形特征（两侧有近距离点，中间有远距离点）
    // 等价 Python: min_dist = np.min(front_dists); min_idx = np.argmin(front_dists)
    auto min_it = std::min_element(front_dists.begin(), front_dists.end());
    size_t min_idx = static_cast<size_t>(std::distance(front_dists.begin(), min_it));
    double min_dist = *min_it;

    detection_.detected = true;
    detection_.distance = min_dist;
    detection_.bearing = front_angles[min_idx];
    detection_.confidence = 0.7;
    detection_.method = "lidar";
  } else {
    detection_.detected = false;
  }
}

void DockDetectorNode::startDocking(
    const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
    std_srvs::srv::Trigger::Response::SharedPtr response) {
  // 启动自动回充
  state_ = DockState::SEARCHING;
  state_start_time_ = this->now();
  response->success = true;
  response->message = "回充已启动";
  RCLCPP_INFO(this->get_logger(), "自动回充已启动");
}

void DockDetectorNode::cancelDocking(
    const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
    std_srvs::srv::Trigger::Response::SharedPtr response) {
  // 取消回充
  state_ = DockState::IDLE;
  geometry_msgs::msg::Twist cmd;
  cmd_vel_pub_->publish(cmd);
  response->success = true;
  response->message = "回充已取消";
}

void DockDetectorNode::controlLoop() {
  // 主控制循环
  if (state_ == DockState::IDLE) {
    return;
  }

  // 发布状态
  std_msgs::msg::String state_msg;
  state_msg.data = stateName(state_);
  state_pub_->publish(state_msg);

  double elapsed = (this->now() - state_start_time_).seconds();

  switch (state_) {
    case DockState::SEARCHING:
      handleSearching(elapsed);
      break;
    case DockState::DETECTED:
      handleDetected();
      break;
    case DockState::APPROACHING:
      handleApproaching();
      break;
    case DockState::ALIGNED:
      handleAligned();
      break;
    case DockState::RECOVERING:
      handleRecovering(elapsed);
      break;
    default:
      break;
  }
}

void DockDetectorNode::handleSearching(double elapsed) {
  // 搜索状态处理
  if (detection_.detected) {
    state_ = DockState::DETECTED;
    state_start_time_ = this->now();
    RCLCPP_INFO(this->get_logger(),
                "检测到dock: 距离=%.2fm 方位=%.1f°",
                detection_.distance,
                detection_.bearing * 180.0 / M_PI);
  } else if (elapsed > search_timeout_) {
    state_ = DockState::FAILED;
    RCLCPP_ERROR(this->get_logger(), "搜索dock超时");
  }
}

void DockDetectorNode::handleDetected() {
  // 检测到dock状态处理
  if (!detection_.detected) {
    state_ = DockState::SEARCHING;
    return;
  }
  state_ = DockState::APPROACHING;
  state_start_time_ = this->now();
}

void DockDetectorNode::handleApproaching() {
  // 接近dock状态处理
  if (!detection_.detected) {
    state_ = DockState::SEARCHING;
    return;
  }

  geometry_msgs::msg::Twist cmd;
  double align_thresh = align_threshold_;
  double dock_dist_thresh = dock_distance_threshold_;

  // 旋转对准dock
  // 等价 Python: cmd.angular.z = 0.5 * np.sign(self._detection.bearing)
  if (std::abs(detection_.bearing) > align_thresh) {
    double sign = (detection_.bearing > 0.0) ? 1.0
                  : (detection_.bearing < 0.0) ? -1.0
                  : 0.0;
    cmd.angular.z = 0.5 * sign;
  }
  // 前进接近
  else if (detection_.distance > dock_dist_thresh) {
    cmd.linear.x = approach_speed_;
  } else {
    // 到达对接距离
    state_ = DockState::ALIGNED;
    RCLCPP_INFO(this->get_logger(), "已对齐dock，准备对接");
  }

  cmd_vel_pub_->publish(cmd);
}

void DockDetectorNode::handleAligned() {
  // 已对齐状态处理
  // 缓慢前进完成对接
  if (detection_.distance > 0.1) {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.05;
    cmd_vel_pub_->publish(cmd);
  } else {
    state_ = DockState::DOCKED;
    geometry_msgs::msg::Twist cmd;
    cmd_vel_pub_->publish(cmd);
    RCLCPP_INFO(this->get_logger(), "对接完成！");
  }
}

void DockDetectorNode::handleRecovering(double elapsed) {
  // 恢复状态处理
  if (elapsed > 5.0) {
    state_ = DockState::SEARCHING;
    state_start_time_ = this->now();
  }
}

std::string DockDetectorNode::stateName(DockState s) {
  // 对应 Python: self.state.name（枚举名）
  switch (s) {
    case DockState::IDLE:        return "IDLE";
    case DockState::SEARCHING:   return "SEARCHING";
    case DockState::DETECTED:    return "DETECTED";
    case DockState::APPROACHING: return "APPROACHING";
    case DockState::ALIGNED:     return "ALIGNED";
    case DockState::DOCKED:      return "DOCKED";
    case DockState::RECOVERING:  return "RECOVERING";
    case DockState::FAILED:      return "FAILED";
  }
  return "UNKNOWN";
}

}  // namespace puppy_core

// ===== 节点入口 =====
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<puppy_core::DockDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
