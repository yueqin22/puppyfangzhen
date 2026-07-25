// Mode Adapter Node 实现
//
// 对应 Python: puppypi_adapter/mode_adapter_node.py
// ROS2 节点，包含 main()
#include "puppypi_adapter/mode_adapter_node.h"

#include <algorithm>
#include <cctype>
#include <stdexcept>

namespace puppypi_adapter {

// 支持的姿态列表
const std::vector<std::string> ModeAdapterNode::POSTURES = {
    "STAND", "SIT", "LIE_DOWN", "RECOVER", "FREEZE",
};

ModeAdapterNode::ModeAdapterNode()
    : rclcpp::Node("mode_adapter") {
    // 参数声明
    this->declare_parameter("use_sim", true);
    use_sim_ = this->get_parameter("use_sim").as_bool();

    // 订阅器
    posture_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/robot/posture_cmd", 10,
        std::bind(&ModeAdapterNode::onPostureCmd, this,
                  std::placeholders::_1));
    enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/robot/motion_enable", 10,
        std::bind(&ModeAdapterNode::onEnable, this,
                  std::placeholders::_1));

    // P0-3: SDK 初始化
    // use_sim=False 时尝试导入 PuppyPi SDK，当前未集成则报错退出
    if (!use_sim_) {
        RCLCPP_FATAL(this->get_logger(),
                     "use_sim=False but PuppyPi SDK unavailable. "
                     "Set use_sim:=true for simulation.");
        throw std::runtime_error("PuppyPi SDK not yet integrated");
    }

    const char* mode_tag = use_sim_ ? "[SIM]" : "[HARDWARE]";
    RCLCPP_INFO(this->get_logger(), "Mode adapter started %s", mode_tag);
}

void ModeAdapterNode::onPostureCmd(
    const std_msgs::msg::String::SharedPtr msg) {
    // 转大写
    std::string posture = msg->data;
    std::transform(posture.begin(), posture.end(), posture.begin(),
                   [](unsigned char c) { return std::toupper(c); });

    // 校验姿态
    bool valid = false;
    for (const auto& p : POSTURES) {
        if (p == posture) {
            valid = true;
            break;
        }
    }
    if (!valid) {
        RCLCPP_WARN(this->get_logger(), "Unknown posture: %s",
                    posture.c_str());
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Posture command: %s",
                posture.c_str());
    // P0-3: 仿真模式只记录，真机模式调用 SDK
    current_posture_ = posture;
    // TODO: 真机模式调用 self.puppypi.set_posture(posture)
}

void ModeAdapterNode::onEnable(
    const std_msgs::msg::Bool::SharedPtr msg) {
    motion_enabled_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "Motors %s",
                msg->data ? "enabled" : "disabled");
    // P0-3: 仿真模式只记录，真机模式调用 SDK
    // TODO: 真机模式调用 self.puppypi.enable_motors(msg->data)
}

}  // namespace puppypi_adapter

// ===== main =====
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<puppypi_adapter::ModeAdapterNode>());
    } catch (const std::exception& e) {
        // 构造失败时 rclcpp::init 已执行，需要 shutdown
        rclcpp::shutdown();
        return 1;
    } catch (...) {
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
