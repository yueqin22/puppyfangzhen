// nav_core ROS2节点实现 (v7.8)
#include "puppy_core/nav_core_node.h"

#include <cmath>
#include <sstream>

namespace puppy_core {

NavCoreNode::NavCoreNode() : rclcpp::Node("nav_core_node") {
  // 延迟导入nav_core模块
  importModules();

  // 参数
  use_amcl_ = this->declare_parameter("use_amcl", true);
  use_teb_ = this->declare_parameter("use_teb", false);
  map_file_ = this->declare_parameter("map_file", std::string(""));
  max_linear_ = this->declare_parameter("max_linear", 0.3);
  max_angular_ = this->declare_parameter("max_angular", 1.2);

  // 初始化算法模块
  initAlgorithmModules();

  // ROS2 接口
  setupSubscribers();
  setupPublishers();
  setupServices();

  // 控制循环定时器 (10Hz)
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&NavCoreNode::controlLoop, this));
  RCLCPP_INFO(this->get_logger(), "nav_core节点已启动 (v7.8)");
}

void NavCoreNode::importModules() {
  // C++版本：nav_core算法后端尚未移植，
  // 保留与Python版相同的"模块加载"语义，方便后续接入C++算法库。
  // 此处默认失败，与Python版"导入失败"路径一致。
  modules_loaded_ = false;
}

void NavCoreNode::initAlgorithmModules() {
  if (!modules_loaded_) {
    return;
  }
  // 占位：在C++算法后端可用时在此实例化
  //   - OccupancyGrid / Costmap / AStarPlanner / DWAPlanner / AMCL / Odometry
  //   - FrontierManager / NavigationRuntimeState / TelemetryCollector
  // Python版根据 use_amcl_/use_teb_ 选择不同规划器，C++版本同样预留分支。
  (void)use_amcl_;
  (void)use_teb_;
}

void NavCoreNode::setupSubscribers() {
  // 设置订阅者
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&NavCoreNode::onScan, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&NavCoreNode::onOdom, this, std::placeholders::_1));
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 1,
      std::bind(&NavCoreNode::onMap, this, std::placeholders::_1));
  initial_pose_sub_ =
      this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/initialpose", 10,
          std::bind(&NavCoreNode::onInitialPose, this, std::placeholders::_1));
}

void NavCoreNode::setupPublishers() {
  // 设置发布者
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10);
  pose_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/amcl_pose", 10);
  status_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/nav_core/status", 10);
}

void NavCoreNode::setupServices() {
  // 设置服务
  start_exploration_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/nav_core/start_exploration",
      std::bind(&NavCoreNode::startExploration, this,
                std::placeholders::_1, std::placeholders::_2));
  stop_nav_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/nav_core/stop",
      std::bind(&NavCoreNode::stopNav, this,
                std::placeholders::_1, std::placeholders::_2));
}

void NavCoreNode::onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  // 处理LiDAR扫描
  if (!modules_loaded_) {
    return;
  }
  // Python版使用 numpy.arange 构造角度向量，
  // C++版本使用 Eigen::VectorXd 替代。
  const int n = static_cast<int>(msg->ranges.size());
  Eigen::VectorXd angles(n);
  for (int i = 0; i < n; ++i) {
    angles(i) = msg->angle_min + i * msg->angle_increment;
  }
  // 距离向量
  Eigen::VectorXd distances(n);
  for (int i = 0; i < n; ++i) {
    distances(i) = msg->ranges[i];
  }
  // 更新占据栅格和代价地图（算法后端未接入，仅保留语义）
  if (has_current_pose_) {
    (void)current_pose_rx_;
    (void)current_pose_ry_;
    (void)current_pose_ryaw_;
    (void)angles;
    (void)distances;
    // self.occ_grid.update_from_scan(rx, ry, angles, distances, max_range=...)
    // self.costmap.update_static(self.occ_grid, frame=self._frame)
  }
}

void NavCoreNode::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
  // 处理里程计，用于AMCL运动更新
  if (!modules_loaded_) {
    return;
  }
  const auto& pose = msg->pose.pose;
  has_last_odom_ = true;
  last_odom_x_ = pose.position.x;
  last_odom_y_ = pose.position.y;
  last_odom_yaw_ = pose.orientation.z;
}

void NavCoreNode::onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  // 处理静态地图
  if (!modules_loaded_) {
    return;
  }
  RCLCPP_INFO(this->get_logger(), "收到地图: %ux%u",
              msg->info.width, msg->info.height);
}

void NavCoreNode::onInitialPose(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  // 处理初始位姿
  if (!modules_loaded_) {
    return;
  }
  const auto& pose = msg->pose.pose;
  // amcl_->recover(pose.position.x, pose.position.y, 0.0, spread=0.5)
  RCLCPP_INFO(this->get_logger(), "设置初始位姿: (%.2f, %.2f)",
              pose.position.x, pose.position.y);
}

void NavCoreNode::startExploration(
    const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
    std_srvs::srv::Trigger::Response::SharedPtr response) {
  // 开始探索服务
  exploring_ = true;
  runtime_state_ = "PLAN";
  response->success = true;
  response->message = "探索已开始";
  RCLCPP_INFO(this->get_logger(), "探索已启动");
}

void NavCoreNode::stopNav(
    const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
    std_srvs::srv::Trigger::Response::SharedPtr response) {
  // 停止导航服务
  exploring_ = false;
  // 停止机器人
  geometry_msgs::msg::Twist cmd;
  cmd_vel_pub_->publish(cmd);
  response->success = true;
  response->message = "导航已停止";
  RCLCPP_INFO(this->get_logger(), "导航已停止");
}

void NavCoreNode::controlLoop() {
  // 主控制循环 (10Hz)
  if (!modules_loaded_ || !exploring_) {
    return;
  }
  ++frame_;
  // 这里简化处理，实际实现需要完整的导航逻辑
  // 发布状态
  std_msgs::msg::String status;
  std::ostringstream oss;
  oss << "frame=" << frame_ << " state=" << runtime_state_;
  status.data = oss.str();
  status_pub_->publish(status);
}

}  // namespace puppy_core
