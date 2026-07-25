// Puppy 机械狗 - 巡逻触发器（thin client）(C++ 版本)
//
// P1-1: 已退化为 mission_manager 的触发入口。
// 不再直接发 Nav2 goal，改为发布 /mission/command 触发 mission_manager 执行巡逻。
// 订阅 /mission/status 显示巡逻进度。
//
// RUNTIME: demo (productization path uses mission_manager as authoritative orchestrator)
// 权威编排层: puppy_core/mission_manager
//
// 用法:
//   ros2 run puppy_nav_cpp patrol            # 触发一次巡逻
//   ros2 run puppy_nav_cpp patrol --ros-args -p stop:=true  # 停止巡逻
//   ros2 launch puppy_bringup full_system.launch.py patrol:=true
//
// 对应 Python: puppy_nav/scripts/patrol.py
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "puppy_interfaces/msg/patrol_status.hpp"

namespace puppy_nav {

/// Thin client: 通过 mission_manager 触发巡逻并监控状态
class PatrolTriggerNode : public rclcpp::Node {
 public:
  /// @param extra_args 额外命令行参数（用于检测 --stop）
  explicit PatrolTriggerNode(const std::vector<std::string>& extra_args = {});
  ~PatrolTriggerNode() override = default;

 private:
  // 等待 publisher 建立连接后发送命令
  void sendCommand();
  // /mission/status 订阅回调
  void onStatus(const puppy_interfaces::msg::PatrolStatus::SharedPtr msg);
  // 停止命令发送后退出节点
  void shutdown();

 private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
  rclcpp::Subscription<puppy_interfaces::msg::PatrolStatus>::SharedPtr
      status_sub_;
  rclcpp::TimerBase::SharedPtr send_timer_;     // 发送命令定时器
  rclcpp::TimerBase::SharedPtr shutdown_timer_;  // 停止后退出定时器

  bool triggered_{false};
  bool stop_mode_{false};  // 是否为停止命令模式
};

}  // namespace puppy_nav
