// Motion Adapter Node 实现
//
// 对应 Python: puppypi_adapter/motion_adapter_node.py
// ROS2 节点，包含 main()
#include "puppypi_adapter/motion_adapter_node.h"

#include <algorithm>
#include <stdexcept>

namespace puppypi_adapter {

const char* MotionAdapterNode::stateName(MotionState s) {
    switch (s) {
        case MotionState::INIT:        return "INIT";
        case MotionState::DISABLED:    return "DISABLED";
        case MotionState::READY:       return "READY";
        case MotionState::EXECUTING:   return "EXECUTING";
        case MotionState::RECOVERING:  return "RECOVERING";
        case MotionState::SAFE_STOP:   return "SAFE_STOP";
        case MotionState::FAULT:        return "FAULT";
    }
    return "UNKNOWN";
}

MotionAdapterNode::MotionAdapterNode()
    : rclcpp::Node("motion_adapter") {
    // 参数声明
    this->declare_parameter("use_sim", true);
    this->declare_parameter("max_linear_x", 0.3);
    this->declare_parameter("max_linear_y", 0.0);
    this->declare_parameter("max_angular_z", 1.2);
    this->declare_parameter("cmd_timeout", 1.0);
    this->declare_parameter("accel_limit", 2.0);
    this->declare_parameter("yaw_rate_limit", 4.0);

    use_sim_ = this->get_parameter("use_sim").as_bool();
    max_linear_x_ = this->get_parameter("max_linear_x").as_double();
    max_linear_y_ = this->get_parameter("max_linear_y").as_double();
    max_angular_z_ = this->get_parameter("max_angular_z").as_double();
    cmd_timeout_ = this->get_parameter("cmd_timeout").as_double();
    accel_limit_ = this->get_parameter("accel_limit").as_double();
    yaw_rate_limit_ = this->get_parameter("yaw_rate_limit").as_double();

    // 初始状态
    platform_state_ = MotionState::INIT;
    last_cmd_time_ = this->now();

    // 发布器
    state_pub_ =
        this->create_publisher<puppy_interfaces::msg::PlatformMotionState>(
            "/platform/motion_state", 10);
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/platform/cmd", 10);

    // 订阅器
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel_safe", 10,
        std::bind(&MotionAdapterNode::onCmdVel, this,
                  std::placeholders::_1));

    // 状态机定时器（20Hz）
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&MotionAdapterNode::updateState, this));

    // P0-3: SDK 初始化（use_sim=False 时尝试导入，失败则报错退出）
    if (!use_sim_) {
        RCLCPP_FATAL(this->get_logger(),
                     "use_sim=False but PuppyPi SDK unavailable. "
                     "Set use_sim:=true for simulation.");
        throw std::runtime_error("PuppyPi SDK not yet integrated");
    }

    const char* mode_tag = use_sim_ ? "[SIM]" : "[HARDWARE]";
    RCLCPP_INFO(this->get_logger(),
                "Motion adapter started %s (state=INIT)", mode_tag);
}

void MotionAdapterNode::onCmdVel(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
    // 不可控或未站立时忽略命令
    if (!controllable_ || !standing_) {
        return;
    }

    last_cmd_time_ = this->now();

    // 速度限制 (gaijin2.md 7.3)
    double target_vx = std::clamp(msg->linear.x, -max_linear_x_, max_linear_x_);
    double target_wz = std::clamp(msg->angular.z, -max_angular_z_, max_angular_z_);

    // 平滑加速度
    const double dt = 0.05;  // 20Hz
    const double max_dv = accel_limit_ * dt;
    const double max_dw = yaw_rate_limit_ * dt;
    current_vx_ = std::clamp(target_vx, current_vx_ - max_dv, current_vx_ + max_dv);
    current_wz_ = std::clamp(target_wz, current_wz_ - max_dw, current_wz_ + max_dw);

    // 死区
    if (std::abs(current_vx_) < 0.01) {
        current_vx_ = 0.0;
    }
    if (std::abs(current_wz_) < 0.05) {
        current_wz_ = 0.0;
    }

    // P0-3: 仿真模式只记录速度；真机模式发送到 SDK
    // TODO: 真机模式 self.puppypi.set_velocity(current_vx_, 0, current_wz_)

    moving_ = (std::abs(current_vx_) > 0.01) || (std::abs(current_wz_) > 0.05);
    if (moving_) {
        platform_state_ = MotionState::EXECUTING;
    }
}

void MotionAdapterNode::updateState() {
    // 命令超时 -> 停止
    rclcpp::Time now = this->now();
    double elapsed = (now - last_cmd_time_).seconds();
    if (controllable_ && standing_ && elapsed > cmd_timeout_) {
        current_vx_ = 0.0;
        current_wz_ = 0.0;
        if (platform_state_ == MotionState::EXECUTING) {
            platform_state_ = MotionState::READY;
            moving_ = false;
        }
    }

    // 状态机转换
    // 仿真模式: INIT -> DISABLED -> READY (自动)
    // 真机模式: 需要 SDK 确认 standing 后才到 READY
    if (platform_state_ == MotionState::INIT) {
        platform_state_ = MotionState::DISABLED;
    } else if (platform_state_ == MotionState::DISABLED) {
        if (use_sim_) {
            // 仿真模式: 自动进入 READY
            platform_state_ = MotionState::READY;
            standing_ = true;
            controllable_ = true;
            gait_mode_ = "stand";
        }
        // 真机模式: 等待 SDK 确认 standing
    } else if (platform_state_ == MotionState::READY && use_sim_) {
        // 仿真模式: 保持 standing 和 controllable
        standing_ = true;
        controllable_ = true;
    }

    publishState();
}

void MotionAdapterNode::publishState() {
    puppy_interfaces::msg::PlatformMotionState msg;
    msg.header.stamp = this->now();
    msg.standing = standing_;
    msg.moving = moving_;
    msg.controllable = controllable_;
    msg.gait_mode = gait_mode_;
    msg.linear_x = current_vx_;
    msg.linear_y = 0.0;
    msg.angular_z = current_wz_;
    msg.platform_state = stateName(platform_state_);
    state_pub_->publish(msg);
}

}  // namespace puppypi_adapter

// ===== main =====
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<puppypi_adapter::MotionAdapterNode>());
    } catch (const std::exception& e) {
        rclcpp::shutdown();
        return 1;
    } catch (...) {
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
