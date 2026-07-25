// CoppeliaSim ZMQ 桥接节点
//
// 完全绕过 simROS2 插件，通过 ZMQ Remote API 直接与 CoppeliaSim 通信
// 发布: /odom, /scan, /clock, /tf (odom->base_footprint)
// 订阅: /cmd_vel
//
// use_sim_time=false（本节点是时钟源，其他 Nav2 节点 use_sim_time=true 从 /clock 获取时间）
//
// 依赖：CoppeliaSim ZMQ Remote API C++ 客户端 (coppeliaSimZMQ.hpp)
//       需要将 CoppeliaSim/programming/zmqRemoteApi/clients/cpp 加入 include 路径
#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/transform_broadcaster.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// CoppeliaSim ZMQ Remote API C++ 客户端
// 提供与 Python 版 RemoteAPIClient 等价的能力
#include "coppeliaSimZMQ.hpp"

namespace puppy_bringup {

/// 轴对齐包围盒（世界坐标）
struct WorldAabb {
  std::array<double, 3> min;
  std::array<double, 3> max;
};

/// CoppeliaSim ZMQ 桥接节点
class CoppeliaBridgeNode : public rclcpp::Node {
 public:
  CoppeliaBridgeNode();
  ~CoppeliaBridgeNode() override = default;

 private:
  // /cmd_vel 回调：缓存速度命令
  void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg);

  // 定时器回调（20Hz）：推进仿真、发布 odom/clock/tf/scan
  void timerCallback();

  // 获取场景中所有 shape 的世界坐标 AABB（排除机器人自身）
  void initObstacles();
  // 计算 shape 的世界坐标轴对齐包围盒
  bool computeWorldAabb(int handle, WorldAabb& out);
  // 计算并发布激光扫描数据
  void publishScan(int32_t sec, int32_t nanosec);

  // Ray-AABB 交集检测（Slab 方法，2D 水平射线 + Z 高度检查）
  // 命中返回 t 值（距离），未命中返回负数
  static double rayAabbIntersect(double ox, double oy, double oz,
                                 double dx, double dy,
                                 const std::array<double, 3>& bmin,
                                 const std::array<double, 3>& bmax,
                                 double max_dist);

 private:
  // ===== CoppeliaSim 连接 =====
  coppeliaSimZMQ sim_;

  // ===== 对象句柄 =====
  int base_footprint_handle_{0};
  int laser_link_handle_{0};
  bool has_laser_link_{false};
  std::array<double, 3> laser_offset_{0.22, 0.0, 0.12};

  // ===== 机器人状态 =====
  double x_{1.0};
  double y_{-2.0};
  double theta_{0.0};
  double vx_{0.0};
  double vtheta_{0.0};

  // ===== 激光雷达参数 =====
  double laser_angle_min_{-M_PI};
  double laser_angle_max_{M_PI};
  double laser_angle_increment_{M_PI / 180.0};  // 1度 = 360点
  double laser_range_min_{0.10};
  double laser_range_max_{12.0};
  int laser_count_{360};

  // 预计算射线角度的 cos/sin（相对机器人）
  std::vector<double> ray_cos_;
  std::vector<double> ray_sin_;

  // ===== cmd_vel 缓存 =====
  double cmd_vel_linear_{0.0};
  double cmd_vel_angular_{0.0};
  std::mutex cmd_vel_lock_;
  double last_cmd_vel_time_{0.0};  // 上次收到 cmd_vel 的墙钟时间（秒）
  double cmd_vel_timeout_{0.5};    // 超过 0.5s 未收到命令则停车

  // ===== 障碍物 =====
  std::vector<WorldAabb> obstacles_;

  // ===== 时间 =====
  double last_sim_time_{0.0};
  int scan_counter_{0};
  int scan_debug_counter_{0};

  // ===== ROS2 发布者 (RELIABLE QoS) =====
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // ===== ROS2 订阅者 =====
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  // ===== 定时器 (20Hz, use_sim_time=false 用墙钟) =====
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace puppy_bringup
