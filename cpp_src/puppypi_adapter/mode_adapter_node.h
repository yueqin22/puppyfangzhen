// Mode Adapter Node: 管理 PuppyPi 姿态/模式切换
//
// 处理姿态命令: STAND, SIT, LIE_DOWN, RECOVER, FREEZE
// (gaijin2.md 第 7.2 节)
//
// P0-3: use_sim=True（默认）时只记录姿态命令不执行；
//       use_sim=False 时尝试导入 PuppyPi SDK，失败则报错退出。
//
// 对应 Python: puppypi_adapter/mode_adapter_node.py
// ROS2 节点，包含 main()
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

namespace puppypi_adapter {

/// 管理 PuppyPi 姿态与模式切换的 ROS2 节点
class ModeAdapterNode : public rclcpp::Node {
public:
    ModeAdapterNode();
    ~ModeAdapterNode() override = default;

private:
    /// 处理姿态命令
    void onPostureCmd(const std_msgs::msg::String::SharedPtr msg);
    /// 处理电机使能/禁用
    void onEnable(const std_msgs::msg::Bool::SharedPtr msg);

private:
    // ===== 参数 =====
    bool use_sim_{true};

    // ===== 状态 =====
    std::string current_posture_{"UNKNOWN"};
    bool motion_enabled_{false};

    // ===== 订阅器 =====
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr posture_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;

    // ===== 支持的姿态列表 =====
    static const std::vector<std::string> POSTURES;
};

}  // namespace puppypi_adapter
