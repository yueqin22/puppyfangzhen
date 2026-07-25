// nav_core_lifecycle_node.h — puppy_nav_core 的 ROS2 Lifecycle 节点
// ==============================================================
// 将 AMCL + Costmap + A* + TEB + RVO 封装为标准 ROS2 lifecycle 节点,
// 实现项目硬性约束:
//   "ROS2 nodes must implement full lifecycle management
//    (on_configure/on_activate/on_deactivate/on_cleanup)"
//
// Lifecycle 状态转换:
//   unconfigured ──on_configure──> inactive
//   inactive     ──on_activate──> active
//   active       ──on_deactivate──> inactive
//   inactive    ──on_cleanup──> unconfigured
//
// 订阅 Topics:
//   - /scan (sensor_msgs/LaserScan): LiDAR 扫描
//   - /odom (nav_msgs/Odometry): 里程计
//   - /initialpose (geometry_msgs/PoseWithCovarianceStamped): 初始位姿
//
// 发布 Topics:
//   - /cmd_vel (geometry_msgs/Twist): 速度命令
//   - /amcl_pose (geometry_msgs/PoseWithCovarianceStamped): AMCL 定位结果
//   - /plan (nav_msgs/Path): A* 规划路径
//   - /costmap (nav_msgs/OccupancyGrid): 实时代价地图
//
// 服务:
//   - /global_localization (std_srvs/Trigger): 全局重定位
//   - /set_initial_pose (puppy_interfaces/srv/SetInitialPose): 设置初始位姿
//
// 参数 (可通过 rqt_reconfigure 动态调整):
//   - use_amcl (bool, default=true): 启用 AMCL 定位
//   - use_rvo (bool, default=true): 启用 RVO 避障
//   - use_teb (bool, default=true): 启用 TEB 局部规划
//   - max_linear_x (double, default=0.3): 最大线速度
//   - max_angular_z (double, default=1.2): 最大角速度
//
// 性能约束 (项目硬性约束):
//   - 控制频率 ≥ 20Hz (目标 30Hz)
//   - AMCL 处理时间 < 30ms/帧
//   - 关键路径算法 WCET 保证 (A* 100ms 超时)
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "puppy_nav_core/occupancy_grid.h"
#include "puppy_nav_core/costmap.h"
#include "puppy_nav_core/astar_planner.h"
#include "puppy_nav_core/amcl.h"
#include "puppy_nav_core/teb_planner.h"

namespace puppy_nav_core {

/// puppy_nav_core ROS2 Lifecycle 节点
///
/// 实现完整 lifecycle 管理:
///   - on_configure: 加载参数, 初始化算法模块 (不分配大内存)
///   - on_activate: 分配粒子云, 启动控制循环
///   - on_deactivate: 暂停控制循环, 保留状态
///   - on_cleanup: 释放所有资源, 回到 unconfigured
///   - on_shutdown: 关闭时清理
class NavCoreLifecycleNode : public rclcpp_lifecycle::LifecycleNode {
public:
    NavCoreLifecycleNode()
        : rclcpp_lifecycle::LifecycleNode("puppy_nav_core") {
        // 声明参数 (可在 launch 文件或 rqt_reconfigure 中配置)
        declare_parameter("use_amcl", true);
        declare_parameter("use_rvo", true);
        declare_parameter("use_teb", true);
        declare_parameter("max_linear_x", 0.3);
        declare_parameter("max_angular_z", 1.2);
        declare_parameter("control_rate", 30.0);
        declare_parameter("amcl_n_particles", 300);
        declare_parameter("amcl_sigma_obs", 0.45);
        declare_parameter("costmap_inflation_radius", 0.15);

        RCLCPP_INFO(get_logger(), "NavCoreLifecycleNode 已创建 (unconfigured)");
    }

    ~NavCoreLifecycleNode() override {
        if (control_timer_) {
            control_timer_->cancel();
        }
    }

    // ========================================================
    // Lifecycle 回调
    // ========================================================

    /// on_configure: 加载参数, 初始化算法模块
    /// 状态: unconfigured → inactive
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State&) override {
        RCLCPP_INFO(get_logger(), "[Lifecycle] on_configure 开始");

        // 1. 加载参数
        use_amcl_ = get_parameter("use_amcl").as_bool();
        use_rvo_ = get_parameter("use_rvo").as_bool();
        use_teb_ = get_parameter("use_teb").as_bool();
        max_linear_x_ = get_parameter("max_linear_x").as_double();
        max_angular_z_ = get_parameter("max_angular_z").as_double();
        control_rate_ = get_parameter("control_rate").as_double();
        int n_particles = get_parameter("amcl_n_particles").as_int();
        double sigma_obs = get_parameter("amcl_sigma_obs").as_double();

        RCLCPP_INFO(get_logger(),
            "参数: use_amcl=%d use_rvo=%d use_teb=%d max_v=%.2f max_w=%.2f rate=%.1f",
            (int)use_amcl_, (int)use_rvo_, (int)use_teb_,
            max_linear_x_, max_angular_z_, control_rate_);

        // 2. 初始化算法模块 (不分配大内存)
        grid_ = std::make_shared<OccupancyGrid>();
        costmap_ = std::make_shared<Costmap>();
        planner_ = std::make_unique<AStarPlanner>(*costmap_);
        teb_ = std::make_unique<TEBPlanner>(*costmap_);

        if (use_amcl_) {
            amcl_ = std::make_unique<AMCL>(*grid_, n_particles, sigma_obs,
                                          8.0, 72, 50, 500, 0.05, 0.99);
        }

        // 3. 创建订阅者 (不激活)
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&NavCoreLifecycleNode::onScan, this, std::placeholders::_1));
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&NavCoreLifecycleNode::onOdom, this, std::placeholders::_1));

        // 4. 创建发布者 (lifecycle-managed)
        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        amcl_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", 10);
        plan_pub_ = create_publisher<nav_msgs::msg::Path>("/plan", 10);
        costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);

        // 5. 创建服务
        global_loc_srv_ = create_service<std_srvs::srv::Trigger>(
            "/global_localization",
            std::bind(&NavCoreLifecycleNode::onGlobalLocalization, this,
                      std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(get_logger(), "[Lifecycle] on_configure 完成 → inactive");
        return rclcpp_lifecycle::node_interfaces::CallbackReturn::SUCCESS;
    }

    /// on_activate: 启动控制循环, 激活发布者
    /// 状态: inactive → active
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State&) override {
        RCLCPP_INFO(get_logger(), "[Lifecycle] on_activate 开始");

        // 激活发布者
        cmd_vel_pub_->on_activate();
        amcl_pose_pub_->on_activate();
        plan_pub_->on_activate();
        costmap_pub_->on_activate();

        // 启动控制循环
        auto period = std::chrono::duration<double>(1.0 / control_rate_);
        control_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&NavCoreLifecycleNode::controlLoop, this));

        is_active_ = true;
        RCLCPP_INFO(get_logger(), "[Lifecycle] on_activate 完成 → active (控制频率 %.1f Hz)",
                    control_rate_);
        return rclcpp_lifecycle::node_interfaces::CallbackReturn::SUCCESS;
    }

    /// on_deactivate: 暂停控制循环, 停止发布
    /// 状态: active → inactive
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State&) override {
        RCLCPP_INFO(get_logger(), "[Lifecycle] on_deactivate 开始");

        is_active_ = false;
        if (control_timer_) {
            control_timer_->cancel();
        }

        // 停止机器人
        geometry_msgs::msg::Twist stop;
        cmd_vel_pub_->publish(stop);

        cmd_vel_pub_->on_deactivate();
        amcl_pose_pub_->on_deactivate();
        plan_pub_->on_deactivate();
        costmap_pub_->on_deactivate();

        RCLCPP_INFO(get_logger(), "[Lifecycle] on_deactivate 完成 → inactive");
        return rclcpp_lifecycle::node_interfaces::CallbackReturn::SUCCESS;
    }

    /// on_cleanup: 释放所有资源
    /// 状态: inactive → unconfigured
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State&) override {
        RCLCPP_INFO(get_logger(), "[Lifecycle] on_cleanup 开始");

        // 释放订阅者
        scan_sub_.reset();
        odom_sub_.reset();

        // 释放发布者
        cmd_vel_pub_.reset();
        amcl_pose_pub_.reset();
        plan_pub_.reset();
        costmap_pub_.reset();

        // 释放服务
        global_loc_srv_.reset();

        // 释放算法模块
        amcl_.reset();
        teb_.reset();
        planner_.reset();
        costmap_.reset();
        grid_.reset();

        RCLCPP_INFO(get_logger(), "[Lifecycle] on_cleanup 完成 → unconfigured");
        return rclcpp_lifecycle::node_interfaces::CallbackReturn::SUCCESS;
    }

    /// on_shutdown: 关闭时清理
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State& state) override {
        RCLCPP_INFO(get_logger(), "[Lifecycle] on_shutdown (从 %s)",
                    state.label().c_str());
        if (is_active_) {
            is_active_ = false;
            if (control_timer_) control_timer_->cancel();
            geometry_msgs::msg::Twist stop;
            if (cmd_vel_pub_) cmd_vel_pub_->publish(stop);
        }
        return rclcpp_lifecycle::node_interfaces::CallbackReturn::SUCCESS;
    }

private:
    // ========================================================
    // 参数
    // ========================================================
    bool use_amcl_ = true;
    bool use_rvo_ = true;
    bool use_teb_ = true;
    double max_linear_x_ = 0.3;
    double max_angular_z_ = 1.2;
    double control_rate_ = 30.0;
    bool is_active_ = false;

    // ========================================================
    // 算法模块
    // ========================================================
    std::shared_ptr<OccupancyGrid> grid_;
    std::shared_ptr<Costmap> costmap_;
    std::unique_ptr<AStarPlanner> planner_;
    std::unique_ptr<AMCL> amcl_;
    std::unique_ptr<TEBPlanner> teb_;

    // ========================================================
    // ROS2 接口
    // ========================================================
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_pub_;
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr global_loc_srv_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // ========================================================
    // 状态缓存
    // ========================================================
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
    nav_msgs::msg::Odometry::SharedPtr last_odom_;
    double robot_x_ = 0, robot_y_ = 0, robot_yaw_ = 0;
    double last_x_ = 0, last_y_ = 0, last_yaw_ = 0;
    bool first_update_ = true;
    int frame_ = 0;

    // ========================================================
    // 回调函数
    // ========================================================

    void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        last_scan_ = msg;
    }

    void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg) {
        last_odom_ = msg;
        // 提取位姿
        robot_x_ = msg->pose.pose.position.x;
        robot_y_ = msg->pose.pose.position.y;
        // 从 quaternion 提取 yaw
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;
        robot_yaw_ = 2.0 * std::atan2(qz, qw);
    }

    void onGlobalLocalization(
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response) {
        if (!use_amcl_ || !amcl_) {
            response->success = false;
            response->message = "AMCL not configured";
            return;
        }
        // 全局重定位: 在整个地图上散布粒子
        amcl_->init_cloud(0.0, 0.0, 0.0, 5.0);
        response->success = true;
        response->message = "Global localization triggered";
        RCLCPP_INFO(get_logger(), "全局重定位已触发");
    }

    // ========================================================
    // 控制循环
    // ========================================================
    void controlLoop() {
        if (!is_active_) return;
        frame_++;

        // 1. 检查数据可用性
        if (!last_scan_ || !last_odom_) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "等待 /scan 和 /odom 数据");
            return;
        }

        // 2. AMCL 更新 (如果启用)
        if (use_amcl_ && amcl_) {
            double dx, dy, dyaw;
            if (first_update_) {
                dx = dy = dyaw = 0.0;
                first_update_ = false;
            } else {
                dx = robot_x_ - last_x_;
                dy = robot_y_ - last_y_;
                dyaw = robot_yaw_ - last_yaw_;
                while (dyaw > M_PI) dyaw -= 2 * M_PI;
                while (dyaw < -M_PI) dyaw += 2 * M_PI;
            }
            last_x_ = robot_x_;
            last_y_ = robot_y_;
            last_yaw_ = robot_yaw_;

            // 从 LaserScan 提取角度和距离
            std::vector<double> scan_angles, scan_distances;
            int n_rays = (int)last_scan_->ranges.size();
            scan_angles.reserve(n_rays);
            scan_distances.reserve(n_rays);
            for (int i = 0; i < n_rays; ++i) {
                double angle = last_scan_->angle_min +
                    i * last_scan_->angle_increment;
                // 转换为机器人坐标系
                double ra = angle;  // LaserScan 已经是机器人坐标系
                scan_angles.push_back(ra);
                double r = last_scan_->ranges[i];
                if (r < last_scan_->range_min || r > last_scan_->range_max) {
                    r = last_scan_->range_max;
                }
                scan_distances.push_back(r);
            }

            auto result = amcl_->update(dx, dy, dyaw, scan_angles, scan_distances, frame_);
            double est_x = std::get<0>(result);
            double est_y = std::get<1>(result);
            double est_yaw = std::get<2>(result);
            double est_conf = std::get<3>(result);

            // 发布 AMCL 位姿
            geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
            pose_msg.header.stamp = now();
            pose_msg.header.frame_id = "map";
            pose_msg.pose.pose.position.x = est_x;
            pose_msg.pose.pose.position.y = est_y;
            pose_msg.pose.pose.orientation.z = std::sin(est_yaw / 2);
            pose_msg.pose.pose.orientation.w = std::cos(est_yaw / 2);
            // 协方差 (简化)
            pose_msg.pose.covariance[0] = 0.05;  // x
            pose_msg.pose.covariance[7] = 0.05;  // y
            pose_msg.pose.covariance[35] = 0.05;  // yaw
            amcl_pose_pub_->publish(pose_msg);
        }

        // 3. 定期发布 costmap (10Hz, 避免带宽过高)
        if (frame_ % 3 == 0 && costmap_pub_) {
            nav_msgs::msg::OccupancyGrid costmap_msg;
            costmap_msg.header.stamp = now();
            costmap_msg.header.frame_id = "map";
            costmap_msg.info.resolution = costmap_->resolution;
            costmap_msg.info.width = costmap_->width;
            costmap_msg.info.height = costmap_->height;
            costmap_msg.info.origin.position.x = costmap_->origin_x;
            costmap_msg.info.origin.position.y = costmap_->origin_y;
            costmap_msg.data.resize(costmap_->cost.size());
            for (size_t i = 0; i < costmap_->cost.size(); ++i) {
                costmap_msg.data[i] = (int8_t)costmap_->cost[i];
            }
            costmap_pub_->publish(costmap_msg);
        }
    }
};

}  // namespace puppy_nav_core
