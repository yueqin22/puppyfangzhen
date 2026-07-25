// Dock感知闭环 (v7.9) (C++ 版本)
//
// 检测充电桩并执行自动回充流程。
// 使用LiDAR和视觉传感器融合检测dock位置。
//
// 状态机:
//   SEARCHING -> DETECTED -> APPROACHING -> ALIGNED -> DOCKED
//   失败时: -> RECOVERING -> SEARCHING
//
// 对应 Python: puppy_core/dock_detector.py
#pragma once

#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace puppy_core {

/// 回充状态（对应 Python DockState(IntEnum)）
enum class DockState : int {
  IDLE = 0,        // 空闲
  SEARCHING = 1,   // 搜索dock
  DETECTED = 2,    // 检测到dock
  APPROACHING = 3, // 接近中
  ALIGNED = 4,     // 已对齐
  DOCKED = 5,      // 已对接
  RECOVERING = 6,  // 恢复中
  FAILED = 7,      // 失败
};

/// Dock检测结果（对应 Python @dataclass DockDetection）
struct DockDetection {
  bool detected{false};
  double distance{0.0};    // 到dock的距离
  double bearing{0.0};     // 方位角(弧度)
  double confidence{0.0};
  std::string method;      // 'lidar', 'vision', 'fusion'
};

/// Dock感知节点 (v7.9)
/// 检测充电桩并执行自动回充
class DockDetectorNode : public rclcpp::Node {
 public:
  DockDetectorNode();
  ~DockDetectorNode() override = default;

 private:
  // 处理LiDAR扫描，检测dock特征
  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  // 启动自动回充服务回调
  void startDocking(
      const std_srvs::srv::Trigger::Request::SharedPtr request,
      std_srvs::srv::Trigger::Response::SharedPtr response);
  // 取消回充服务回调
  void cancelDocking(
      const std_srvs::srv::Trigger::Request::SharedPtr request,
      std_srvs::srv::Trigger::Response::SharedPtr response);
  // 主控制循环
  void controlLoop();

  // 状态处理函数
  void handleSearching(double elapsed);
  void handleDetected();
  void handleApproaching();
  void handleAligned();
  void handleRecovering(double elapsed);

  // 获取状态名称（用于发布）
  static std::string stateName(DockState s);

 private:
  // ===== 参数 =====
  double dock_x_{0.0};
  double dock_y_{-3.5};
  double dock_yaw_{0.0};
  double approach_speed_{0.1};
  double align_threshold_{0.1};        // 对齐阈值(弧度)
  double dock_distance_threshold_{0.3}; // 对接距离
  double search_timeout_{30.0};        // 搜索超时(秒)

  // ===== 状态 =====
  DockState state_{DockState::IDLE};
  DockDetection detection_;
  rclcpp::Time state_start_time_;

  // ===== ROS2接口 =====
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr cancel_srv_;

  // ===== 控制循环定时器 =====
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace puppy_core
