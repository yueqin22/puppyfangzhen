// 定时任务调度节点（C++ 版本）
//
// - 早安巡逻 (7:00): 打开窗帘，全屋巡逻，早安问候
// - 午间检查 (12:00): 快速巡检，服药提醒
// - 晚间安防 (22:00): 夜间模式，安防巡逻，关灯关窗帘
//
// 支持加速仿真模式: 每个真实分钟 = 1仿真小时
//   即 7:00 → 运行后7分钟触发
//      12:00 → 运行后12分钟触发
//      22:00 → 运行后22分钟触发
//
// 也支持手动触发: /task_scheduler/cmd "morning" / "noon" / "night"
#pragma once

#include <chrono>
#include <memory>
#include <set>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace puppy_gait {

/// 定时任务调度节点
class TaskSchedulerNode : public rclcpp::Node {
 public:
  TaskSchedulerNode();
  ~TaskSchedulerNode() override = default;

 private:
  // 获取当前仿真小时
  int getCurrentHour() const;
  // 检查是否需要触发定时任务
  void checkSchedule();
  // 早安任务
  void triggerMorning();
  // 午间任务
  void triggerNoon();
  // 晚间任务
  void triggerNight();
  // 手动触发任务
  void cmdCallback(const std_msgs::msg::String::SharedPtr msg);
  // 发布语音合成文本
  void publishVoice(const std::string &text);

 private:
  // ===== 参数 =====
  bool accelerated_{true};
  int morning_hour_{7};
  int noon_hour_{12};
  int night_hour_{22};

  // ===== 状态 =====
  std::set<std::string> completed_tasks_;  // 已完成的任务（防止重复触发）
  int sim_day_{0};                         // 仿真天数
  int last_sim_hour_{-1};

  // ===== 订阅 =====
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;

  // ===== 发布 =====
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr voice_pub_;

  // ===== 定时器 =====
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace puppy_gait
