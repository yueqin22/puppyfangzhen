// Puppy 机械狗自动巡航节点（C++ 版本）
//
// 在家庭环境中自动巡航，沿预设路径行走并建图
// 路径设计（家庭环境 home.world）：
//   起点在客厅中央 (0, -2)
//   巡航路线：客厅 -> 走廊 -> 卧室 -> 走廊 -> 厨房 -> 客厅
#pragma once

#include <memory>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace puppy_gait {

/// 自动巡航节点：沿预设步骤发布速度命令，完成全屋巡航
class AutoCruiseNode : public rclcpp::Node {
 public:
  AutoCruiseNode();
  ~AutoCruiseNode() override = default;

 private:
  // 定时回调：按步骤发布速度命令
  void timerCallback();

  // 单步巡航定义：持续时间、线速度、角速度、描述
  struct CruiseStep {
    double duration_sec;
    double linear_x;
    double angular_z;
    std::string description;
  };

 private:
  // 速度命令发布器
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  // 控制定时器（20Hz）
  rclcpp::TimerBase::SharedPtr timer_;

  // 巡航步骤列表
  std::vector<CruiseStep> steps_;
  // 当前步骤索引
  size_t current_step_{0};
  // 当前步骤开始时间
  rclcpp::Time state_start_time_;
};

}  // namespace puppy_gait
