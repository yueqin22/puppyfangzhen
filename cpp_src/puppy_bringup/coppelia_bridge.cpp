// CoppeliaSim ZMQ 桥接节点实现
#include "puppy_bringup/coppelia_bridge.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>

namespace puppy_bringup {

namespace {

// 获取当前墙钟时间（秒），等价 Python time.time()
double wallNowSeconds() {
  auto now = std::chrono::steady_clock::now();
  auto duration = now.time_since_epoch();
  return std::chrono::duration<double>(duration).count();
}

}  // namespace

CoppeliaBridgeNode::CoppeliaBridgeNode()
    : rclcpp::Node("coppelia_bridge") {
  // ===== 连接 CoppeliaSim =====
  RCLCPP_INFO(this->get_logger(), "连接 CoppeliaSim ZMQ...");
  // coppeliaSimZMQ 构造时即建立 ZMQ 连接
  RCLCPP_INFO(this->get_logger(), "ZMQ 连接成功!");

  // ===== 获取对象句柄 =====
  base_footprint_handle_ = sim_.getObject("/base_footprint");

  // 读取激光雷达相对偏移（相对 base_footprint）
  try {
    laser_link_handle_ = sim_.getObject("/base_footprint/base_link/laser_link");
    std::vector<double> laser_pos =
        sim_.getObjectPosition(laser_link_handle_, base_footprint_handle_);
    laser_offset_ = {laser_pos[0], laser_pos[1], laser_pos[2]};
    has_laser_link_ = true;
    RCLCPP_INFO(this->get_logger(),
                "激光雷达偏移(相对base_footprint): [%.3f, %.3f, %.3f]",
                laser_offset_[0], laser_offset_[1], laser_offset_[2]);
  } catch (const std::exception& e) {
    RCLCPP_WARN(this->get_logger(),
                "获取激光雷达句柄失败，使用默认偏移: %s", e.what());
    laser_offset_ = {0.22, 0.0, 0.12};
  }

  // ===== 机器人状态 =====
  // 设置初始位置
  sim_.setObjectPosition(base_footprint_handle_, -1,
                         std::vector<double>{x_, y_, 0.0});
  sim_.setObjectOrientation(base_footprint_handle_, -1,
                            std::vector<double>{0.0, 0.0, theta_});

  // ===== 预计算射线角度的 cos/sin（相对机器人）=====
  ray_cos_.resize(laser_count_);
  ray_sin_.resize(laser_count_);
  for (int i = 0; i < laser_count_; ++i) {
    double angle = laser_angle_min_ + i * laser_angle_increment_;
    ray_cos_[i] = std::cos(angle);
    ray_sin_[i] = std::sin(angle);
  }

  // ===== cmd_vel 缓存 =====
  last_cmd_vel_time_ = wallNowSeconds();

  // ===== 计算障碍物 =====
  initObstacles();
  RCLCPP_INFO(this->get_logger(), "障碍物数量: %zu", obstacles_.size());

  // ===== 时间 =====
  last_sim_time_ = sim_.getSimulationTime();

  // ===== ROS2 发布者 (RELIABLE QoS) =====
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
  clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  // ===== ROS2 订阅者 =====
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&CoppeliaBridgeNode::onCmdVel, this, std::placeholders::_1));

  // ===== 定时器 (20Hz, use_sim_time=false 用墙钟) =====
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&CoppeliaBridgeNode::timerCallback, this));

  RCLCPP_INFO(this->get_logger(),
              "CoppeliaSim ZMQ 桥接节点已启动 (20Hz odom, 10Hz scan)");
}

void CoppeliaBridgeNode::onCmdVel(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(cmd_vel_lock_);
  cmd_vel_linear_ = msg->linear.x;
  cmd_vel_angular_ = msg->angular.z;
  last_cmd_vel_time_ = wallNowSeconds();
}

void CoppeliaBridgeNode::initObstacles() {
  // 获取场景中所有 shape 的世界坐标 AABB
  std::vector<int> shapes = sim_.getObjectsInTree(
      sim_.handle_scene, sim_.object_shape_type, 0);

  // 获取机器人自身的所有子对象（需要排除）
  std::vector<int> robot_tree = sim_.getObjectsInTree(
      base_footprint_handle_, sim_.object_shape_type, 0);
  // 用集合加速查找
  std::vector<bool> is_robot(robot_tree.size(), false);
  auto isRobotShape = [&](int handle) {
    for (int h : robot_tree) {
      if (h == handle) return true;
    }
    return false;
  };

  for (int handle : shapes) {
    if (!isRobotShape(handle)) {
      WorldAabb aabb;
      if (computeWorldAabb(handle, aabb)) {
        obstacles_.push_back(aabb);
      }
    }
  }
}

bool CoppeliaBridgeNode::computeWorldAabb(int handle, WorldAabb& out) {
  // 计算 shape 的世界坐标轴对齐包围盒 (AABB)
  double min_x, max_x, min_y, max_y, min_z, max_z;
  try {
    min_x = sim_.getObjectFloatParam(handle, sim_.objfloatparam_objbbox_min_x);
    max_x = sim_.getObjectFloatParam(handle, sim_.objfloatparam_objbbox_max_x);
    min_y = sim_.getObjectFloatParam(handle, sim_.objfloatparam_objbbox_min_y);
    max_y = sim_.getObjectFloatParam(handle, sim_.objfloatparam_objbbox_max_y);
    min_z = sim_.getObjectFloatParam(handle, sim_.objfloatparam_objbbox_min_z);
    max_z = sim_.getObjectFloatParam(handle, sim_.objfloatparam_objbbox_max_z);
  } catch (const std::exception&) {
    return false;
  }

  // 8 个角点（本地坐标）
  std::array<std::array<double, 3>, 8> corners = {{
      {{min_x, min_y, min_z}}, {{max_x, min_y, min_z}},
      {{min_x, max_y, min_z}}, {{max_x, max_y, min_z}},
      {{min_x, min_y, max_z}}, {{max_x, min_y, max_z}},
      {{min_x, max_y, max_z}}, {{max_x, max_y, max_z}},
  }};

  // 世界变换矩阵
  std::vector<double> matrix = sim_.getObjectMatrix(handle, -1);

  // 变换角点到世界坐标，计算 AABB
  std::array<double, 3> world_min = {
      std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::infinity()};
  std::array<double, 3> world_max = {
      -std::numeric_limits<double>::infinity(),
      -std::numeric_limits<double>::infinity(),
      -std::numeric_limits<double>::infinity()};

  for (const auto& corner : corners) {
    std::vector<double> wv = sim_.multiplyVector(
        matrix, std::vector<double>{corner[0], corner[1], corner[2]});
    for (int j = 0; j < 3; ++j) {
      if (wv[j] < world_min[j]) world_min[j] = wv[j];
      if (wv[j] > world_max[j]) world_max[j] = wv[j];
    }
  }

  out.min = world_min;
  out.max = world_max;
  return true;
}

double CoppeliaBridgeNode::rayAabbIntersect(
    double ox, double oy, double oz, double dx, double dy,
    const std::array<double, 3>& bmin, const std::array<double, 3>& bmax,
    double max_dist) {
  // Ray-AABB 交集检测（Slab 方法，2D 水平射线 + Z 高度检查）
  // 先检查 Z 高度：激光是否在障碍物 Z 范围内
  if (oz < bmin[2] || oz > bmax[2]) {
    return -1.0;  // 未命中
  }

  double tmin = 0.0;
  double tmax = max_dist;

  // X 轴
  if (std::abs(dx) > 1e-10) {
    double t1 = (bmin[0] - ox) / dx;
    double t2 = (bmax[0] - ox) / dx;
    if (t1 > t2) std::swap(t1, t2);
    if (t1 > tmin) tmin = t1;
    if (t2 < tmax) tmax = t2;
    if (tmin > tmax) return -1.0;
  } else if (ox < bmin[0] || ox > bmax[0]) {
    return -1.0;
  }

  // Y 轴
  if (std::abs(dy) > 1e-10) {
    double t1 = (bmin[1] - oy) / dy;
    double t2 = (bmax[1] - oy) / dy;
    if (t1 > t2) std::swap(t1, t2);
    if (t1 > tmin) tmin = t1;
    if (t2 < tmax) tmax = t2;
    if (tmin > tmax) return -1.0;
  } else if (oy < bmin[1] || oy > bmax[1]) {
    return -1.0;
  }

  return tmin;
}

void CoppeliaBridgeNode::timerCallback() {
  try {
    // 读取仿真时间
    double current_time = sim_.getSimulationTime();
    double dt = current_time - last_sim_time_;
    if (dt <= 0) {
      return;  // 仿真未推进，跳过
    }
    if (dt > 0.1) {
      dt = 0.1;  // 限制最大步长
    }
    last_sim_time_ = current_time;

    // 获取 cmd_vel（带超时保护）
    double vx, vth;
    {
      std::lock_guard<std::mutex> lock(cmd_vel_lock_);
      vx = cmd_vel_linear_;
      vth = cmd_vel_angular_;
      // 超过 0.5s 未收到新命令则停车（避免通信中断时失控）
      if (wallNowSeconds() - last_cmd_vel_time_ > cmd_vel_timeout_) {
        vx = 0.0;
        vth = 0.0;
      }
    }

    // 差速驱动运动模型
    x_ += vx * std::cos(theta_) * dt;
    y_ += vx * std::sin(theta_) * dt;
    theta_ += vth * dt;

    // 限制在地图范围内
    x_ = std::max(-4.9, std::min(4.9, x_));
    y_ = std::max(-3.9, std::min(3.9, y_));

    // 更新机器人位置（ZMQ 写入 CoppeliaSim）
    sim_.setObjectPosition(base_footprint_handle_, -1,
                           std::vector<double>{x_, y_, 0.0});
    sim_.setObjectOrientation(base_footprint_handle_, -1,
                              std::vector<double>{0.0, 0.0, theta_});

    vx_ = vx;
    vtheta_ = vth;

    // 时间戳
    int32_t sec = static_cast<int32_t>(current_time);
    int32_t nanosec = static_cast<int32_t>(
        std::fmod(current_time, 1.0) * 1e9);

    // 发布 /clock
    rosgraph_msgs::msg::Clock clock_msg;
    clock_msg.clock.sec = sec;
    clock_msg.clock.nanosec = nanosec;
    clock_pub_->publish(clock_msg);

    // 发布 /odom
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp.sec = sec;
    odom_msg.header.stamp.nanosec = nanosec;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_footprint";
    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation.z = std::sin(theta_ / 2.0);
    odom_msg.pose.pose.orientation.w = std::cos(theta_ / 2.0);
    odom_msg.pose.covariance.fill(0.0);
    odom_msg.twist.twist.linear.x = vx_;
    odom_msg.twist.twist.angular.z = vtheta_;
    odom_msg.twist.covariance.fill(0.0);
    odom_pub_->publish(odom_msg);

    // 发布 /tf (odom -> base_footprint)
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp.sec = sec;
    t.header.stamp.nanosec = nanosec;
    t.header.frame_id = "odom";
    t.child_frame_id = "base_footprint";
    t.transform.translation.x = x_;
    t.transform.translation.y = y_;
    t.transform.translation.z = 0.0;
    t.transform.rotation.z = std::sin(theta_ / 2.0);
    t.transform.rotation.w = std::cos(theta_ / 2.0);
    tf_broadcaster_->sendTransform(t);

    // 发布 /scan (每 2 个周期 = 10Hz)
    scan_counter_ += 1;
    if (scan_counter_ >= 2) {
      scan_counter_ = 0;
      publishScan(sec, nanosec);
    }
  } catch (const std::exception& e) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                         "定时器回调异常: %s", e.what());
  }
}

void CoppeliaBridgeNode::publishScan(int32_t sec, int32_t nanosec) {
  // 计算并发布激光扫描数据
  double t0 = wallNowSeconds();

  // 计算激光雷达世界坐标位置
  double cos_th = std::cos(theta_);
  double sin_th = std::sin(theta_);
  double ox = x_ + laser_offset_[0] * cos_th - laser_offset_[1] * sin_th;
  double oy = y_ + laser_offset_[0] * sin_th + laser_offset_[1] * cos_th;
  double oz = laser_offset_[2];

  std::vector<float> ranges(laser_count_,
                            static_cast<float>(laser_range_max_));

  for (int i = 0; i < laser_count_; ++i) {
    // 旋转射线方向（预计算的相对角度 + 机器人朝向）
    double dx = ray_cos_[i] * cos_th - ray_sin_[i] * sin_th;
    double dy = ray_cos_[i] * sin_th + ray_sin_[i] * cos_th;

    double min_dist = laser_range_max_;
    for (const auto& aabb : obstacles_) {
      double dist = rayAabbIntersect(ox, oy, oz, dx, dy,
                                     aabb.min, aabb.max, laser_range_max_);
      if (dist >= 0.0 && dist < min_dist) {
        min_dist = dist;
      }
    }

    if (min_dist < laser_range_min_) {
      min_dist = laser_range_min_;
    }
    ranges[i] = static_cast<float>(min_dist);
  }

  sensor_msgs::msg::LaserScan scan_msg;
  scan_msg.header.stamp.sec = sec;
  scan_msg.header.stamp.nanosec = nanosec;
  scan_msg.header.frame_id = "laser_link";
  scan_msg.angle_min = laser_angle_min_;
  scan_msg.angle_max = laser_angle_max_;
  scan_msg.angle_increment = laser_angle_increment_;
  scan_msg.time_increment = 0.0;
  scan_msg.scan_time = 0.1;
  scan_msg.range_min = laser_range_min_;
  scan_msg.range_max = laser_range_max_;
  scan_msg.ranges = ranges;

  // 订阅者数量
  size_t num_subs = scan_pub_->get_subscription_count();
  scan_pub_->publish(scan_msg);

  // 每 50 次（约 5 秒）输出一次调试日志
  scan_debug_counter_ += 1;
  if (scan_debug_counter_ >= 50) {
    scan_debug_counter_ = 0;
    double elapsed_ms = (wallNowSeconds() - t0) * 1000.0;
    // 找最近的障碍物距离
    float min_range = ranges.empty() ? 0.0f : ranges[0];
    for (float r : ranges) {
      if (r < min_range) min_range = r;
    }
    RCLCPP_INFO(this->get_logger(),
                "/scan 发布: %d点, 耗时%.1fms, 订阅者=%zu, 最近障碍=%.2fm",
                laser_count_, elapsed_ms, num_subs,
                static_cast<double>(min_range));
  }
}

}  // namespace puppy_bringup
