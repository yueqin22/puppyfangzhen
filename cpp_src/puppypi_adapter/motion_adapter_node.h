// Motion Adapter Node: 将 /cmd_vel 转换为 PuppyPi 运动命令
//
// 状态机: INIT -> DISABLED -> READY -> EXECUTING -> RECOVERING -> SAFE_STOP -> FAULT
// (gaijin2.md 第 5.3 节)
//
// 输入:
//   - /cmd_vel_safe (geometry_msgs/Twist)
// 输出:
//   - /platform/cmd (geometry_msgs/Twist) — 转换后的运动命令
//   - /platform/motion_state (PlatformMotionState)
//
// 对应 Python: puppypi_adapter/motion_adapter_node.py
// ROS2 节点，包含 main()
#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "puppy_interfaces/msg/platform_motion_state.hpp"

namespace puppypi_adapter {

/// 运动状态枚举（对应 Python: MotionState 常量）
enum class MotionState {
    INIT,
    DISABLED,
    READY,
    EXECUTING,
    RECOVERING,
    SAFE_STOP,
    FAULT,
};

/// 将速度命令转换为 PuppyPi 平台动作的 ROS2 节点
class MotionAdapterNode : public rclcpp::Node {
public:
    MotionAdapterNode();
    ~MotionAdapterNode() override = default;

private:
    /// 处理输入速度命令（含安全处理）
    void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg);
    /// 状态机更新与超时处理（20Hz）
    void updateState();
    /// 发布平台运动状态
    void publishState();

    /// 将 MotionState 转为字符串
    static const char* stateName(MotionState s);

private:
    // ===== 参数 =====
    bool use_sim_{true};
    double max_linear_x_{0.3};
    double max_linear_y_{0.3};
    double max_angular_z_{1.2};
    double cmd_timeout_{1.0};
    double accel_limit_{2.0};
    double yaw_rate_limit_{4.0};

    // ===== 平台状态 =====
    MotionState platform_state_{MotionState::INIT};
    bool standing_{false};
    bool moving_{false};
    bool controllable_{false};
    std::string gait_mode_{"idle"};

    // ===== 平滑后的速度 =====
    // P0-3: vy 与 vx 同等对待。此前只有 vx/wz, linear.y 在整条链路上被丢弃,
    // 而平台实测横移可兑现 82%~118% (config/motion_capability.yaml),
    // 导航的受阻切向脱困正依赖它。
    double current_vx_{0.0};
    double current_vy_{0.0};
    double current_wz_{0.0};

    // ===== 上次命令时间 =====
    rclcpp::Time last_cmd_time_;

    // ===== 发布器 =====
    rclcpp::Publisher<puppy_interfaces::msg::PlatformMotionState>::SharedPtr
        state_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    // ===== 订阅器 =====
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    // ===== 定时器（20Hz 状态机更新）=====
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace puppypi_adapter
