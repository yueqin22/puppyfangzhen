// 定时任务调度节点实现
#include "puppy_gait/task_scheduler.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <string>

namespace puppy_gait {

TaskSchedulerNode::TaskSchedulerNode()
    : rclcpp::Node("task_scheduler") {
  // ===== 参数 =====
  this->declare_parameter("accelerated_time", true);
  this->declare_parameter("morning_hour", 7);
  this->declare_parameter("noon_hour", 12);
  this->declare_parameter("night_hour", 22);
  accelerated_ = this->get_parameter("accelerated_time").as_bool();
  morning_hour_ = this->get_parameter("morning_hour").as_int();
  noon_hour_ = this->get_parameter("noon_hour").as_int();
  night_hour_ = this->get_parameter("night_hour").as_int();

  // ===== 状态 =====
  sim_day_ = 0;
  last_sim_hour_ = -1;

  // ===== 订阅 =====
  cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/task_scheduler/cmd", 10,
      std::bind(&TaskSchedulerNode::cmdCallback, this, std::placeholders::_1));

  // ===== 发布 =====
  event_pub_ =
      this->create_publisher<std_msgs::msg::String>("/task_scheduler/event", 10);
  voice_pub_ = this->create_publisher<std_msgs::msg::String>("/voice/tts", 10);

  // ===== 定时器 =====
  // 每10秒检查一次
  timer_ = this->create_wall_timer(
      std::chrono::seconds(10),
      std::bind(&TaskSchedulerNode::checkSchedule, this));

  RCLCPP_INFO(this->get_logger(),
              "定时任务调度节点已启动 (加速模式: %s)",
              accelerated_ ? "true" : "false");
  RCLCPP_INFO(this->get_logger(),
              "计划: 早安%d:00, 午间%d:00, 晚间%d:00", morning_hour_,
              noon_hour_, night_hour_);
}

int TaskSchedulerNode::getCurrentHour() const {
  // 获取当前仿真小时
  // 加速模式: 运行时间(分钟) % 24 = 仿真小时
  // 正常模式: 实际时间小时
  if (accelerated_) {
    double total_sec = this->get_clock()->now().seconds();
    // 每60秒 = 1仿真小时
    int sim_hour = static_cast<int>(std::fmod(total_sec / 60.0, 24.0));
    return sim_hour;
  } else {
    std::time_t t = std::time(nullptr);
    std::tm *lt = std::localtime(&t);
    return lt->tm_hour;
  }
}

void TaskSchedulerNode::checkSchedule() {
  // 检查是否需要触发定时任务
  int current_hour = getCurrentHour();

  // 检测新的一天（小时从大变小）
  if (current_hour < last_sim_hour_) {
    sim_day_++;
    completed_tasks_.clear();
    RCLCPP_INFO(this->get_logger(), "新的一天开始 (第%d天)", sim_day_ + 1);
  }

  last_sim_hour_ = current_hour;

  // 生成任务标识（天+小时），防止重复触发
  std::string task_base = "day" + std::to_string(sim_day_);

  // 早安巡逻
  if (current_hour == morning_hour_) {
    std::string task_id = task_base + "_morning";
    if (completed_tasks_.find(task_id) == completed_tasks_.end()) {
      completed_tasks_.insert(task_id);
      triggerMorning();
    }
  }
  // 午间检查
  else if (current_hour == noon_hour_) {
    std::string task_id = task_base + "_noon";
    if (completed_tasks_.find(task_id) == completed_tasks_.end()) {
      completed_tasks_.insert(task_id);
      triggerNoon();
    }
  }
  // 晚间安防
  else if (current_hour == night_hour_) {
    std::string task_id = task_base + "_night";
    if (completed_tasks_.find(task_id) == completed_tasks_.end()) {
      completed_tasks_.insert(task_id);
      triggerNight();
    }
  }
}

void TaskSchedulerNode::triggerMorning() {
  // 早安任务
  RCLCPP_INFO(this->get_logger(), "===== 触发早安任务 =====");
  std_msgs::msg::String event;
  event.data = "morning_patrol";
  event_pub_->publish(event);
  publishVoice(
      "早安主人！现在是早上7点，我来开启全屋巡逻。窗帘已打开，祝您一天好心情");
}

void TaskSchedulerNode::triggerNoon() {
  // 午间任务
  RCLCPP_INFO(this->get_logger(), "===== 触发午间任务 =====");
  std_msgs::msg::String event;
  event.data = "noon_check";
  event_pub_->publish(event);
  publishVoice(
      "午间巡检时间到了。主人，别忘了吃午饭和服药哦，我来检查一下家里的安全状况");
}

void TaskSchedulerNode::triggerNight() {
  // 晚间任务
  RCLCPP_INFO(this->get_logger(), "===== 触发晚间安防任务 =====");
  std_msgs::msg::String event;
  event.data = "night_patrol";
  event_pub_->publish(event);
  publishVoice(
      "晚间安防模式已启动。窗帘已关闭，灯光已调暗，我来做最后一次安全巡逻，"
      "主人晚安");
}

void TaskSchedulerNode::cmdCallback(
    const std_msgs::msg::String::SharedPtr msg) {
  // 手动触发任务
  std::string cmd = msg->data;
  // 转小写
  std::transform(cmd.begin(), cmd.end(), cmd.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  RCLCPP_INFO(this->get_logger(), "收到手动触发命令: %s", cmd.c_str());

  if (cmd == "morning" || cmd == "早安" || cmd == "早上") {
    triggerMorning();
  } else if (cmd == "noon" || cmd == "午间" || cmd == "中午") {
    triggerNoon();
  } else if (cmd == "night" || cmd == "晚间" || cmd == "晚上" ||
             cmd == "晚安") {
    triggerNight();
  } else if (cmd == "status") {
    int current_hour = getCurrentHour();
    std::string status = "当前仿真时间: 第" + std::to_string(sim_day_ + 1) +
                         "天 " + std::to_string(current_hour) +
                         ":00, 已完成任务: " +
                         std::to_string(completed_tasks_.size()) + "个";
    publishVoice(status);
    RCLCPP_INFO(this->get_logger(), "%s", status.c_str());
  } else {
    RCLCPP_WARN(this->get_logger(),
                "未知命令: %s（支持: morning/noon/night/status）",
                cmd.c_str());
  }
}

void TaskSchedulerNode::publishVoice(const std::string &text) {
  std_msgs::msg::String msg;
  msg.data = text;
  voice_pub_->publish(msg);
}

}  // namespace puppy_gait
