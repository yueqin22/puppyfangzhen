// nav_core ROS2节点 (v7.8)
//
// 将nav_core的核心导航算法（AMCL、A*、DWA/TEB、frontier探索）
// 包装为ROS2节点，使研究成果可以在ROS2产品系统中使用。
//
// 订阅:
//   - /scan (sensor_msgs/LaserScan): LiDAR扫描
//   - /odom (nav_msgs/Odometry): 里程计
//   - /map (nav_msgs/OccupancyGrid): 静态地图
//   - /initialpose (geometry_msgs/PoseWithCovarianceStamped): 初始位姿
//
// 发布:
//   - /cmd_vel (geometry_msgs/Twist): 速度命令
//   - /amcl_pose (geometry_msgs/PoseWithCovarianceStamped): 定位结果
//   - /frontiers (visualization_msgs/MarkerArray): frontier可视化
//
// 服务:
//   - /nav_core/start_exploration: 开始探索
//   - /nav_core/stop: 停止导航
#pragma once

#include <memory>
#include <string>
#include <vector>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace puppy_core {

/// nav_core ROS2包装节点 (v7.8)
///
/// 将纯算法的nav_core包装为ROS2节点，
/// 使产品系统可以通过标准ROS2接口使用研究成果。
class NavCoreNode : public rclcpp::Node {
 public:
  NavCoreNode();
  ~NavCoreNode() override = default;

 private:
  // 延迟导入nav_core模块（C++版本：尝试加载算法后端，失败则禁用）
  void importModules();

  // 初始化算法模块实例
  void initAlgorithmModules();

  // 设置订阅者
  void setupSubscribers();
  // 设置发布者
  void setupPublishers();
  // 设置服务
  void setupServices();

  // 回调
  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void onInitialPose(
      const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

  // 服务回调
  void startExploration(
      const std_srvs::srv::Trigger::Request::SharedPtr request,
      std_srvs::srv::Trigger::Response::SharedPtr response);
  void stopNav(
      const std_srvs::srv::Trigger::Request::SharedPtr request,
      std_srvs::srv::Trigger::Response::SharedPtr response);

  // 主控制循环 (10Hz)
  void controlLoop();

 private:
  // 参数
  bool use_amcl_;
  bool use_teb_;
  std::string map_file_;
  double max_linear_;
  double max_angular_;

  // 算法模块加载标志（C++后端尚未实现，保留接口与Python一致）
  bool modules_loaded_{false};

  // 状态
  bool exploring_{false};
  bool has_current_pose_{false};
  double current_pose_rx_{0.0};
  double current_pose_ry_{0.0};
  double current_pose_ryaw_{0.0};
  int64_t frame_{0};

  // 最近里程计（用于AMCL运动更新）
  bool has_last_odom_{false};
  double last_odom_x_{0.0};
  double last_odom_y_{0.0};
  double last_odom_yaw_{0.0};

  // 运行时状态（Python版使用NavState枚举，C++版用字符串简化）
  std::string runtime_state_{"IDLE"};

  // ROS2 接口
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      initial_pose_sub_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      pose_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_exploration_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_nav_srv_;

  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace puppy_core
