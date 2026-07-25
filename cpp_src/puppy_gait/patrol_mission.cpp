// 家庭巡逻任务节点实现
#include "puppy_gait/patrol_mission.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>

namespace puppy_gait {

PatrolMissionNode::PatrolMissionNode()
    : rclcpp::Node("patrol_mission") {
  // 导航客户端
  nav_client_ = rclcpp_action::create_client<NavigateToPose>(
      this, "navigate_to_pose");

  // 发布器
  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  alert_pub_ =
      this->create_publisher<std_msgs::msg::String>("/patrol/alert", 10);
  status_pub_ =
      this->create_publisher<std_msgs::msg::String>("/patrol/status", 10);
  emotion_pub_ =
      this->create_publisher<std_msgs::msg::String>("/emotion/command", 10);

  // 订阅器
  battery_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
      "/battery_state", 10,
      std::bind(&PatrolMissionNode::batteryCallback, this,
                std::placeholders::_1));
  fall_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/fall_detected", 10,
      std::bind(&PatrolMissionNode::fallCallback, this, std::placeholders::_1));
  emotion_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/emotion/recognized", 10,
      std::bind(&PatrolMissionNode::emotionCallback, this,
                std::placeholders::_1));

  // 状态
  battery_level_ = 1.0;
  is_charging_ = false;
  patrol_count_ = 0;
  current_waypoint_ = 0;
  is_navigating_ = false;
  emergency_stop_ = false;
  state_ = PatrolState::Idle;

  // 充电桩位置（起点）— 从 puppy_core/config/navigation_targets.yaml 加载
  charging_dock_ = {"dock", 0.0, -2.0, 3.14159};

  // P0-5: 巡逻路径点从 puppy_core/config/patrol_routes.yaml 加载
  // 替代原硬编码 waypoints，统一坐标来源
  // 此处使用配置文件中 default 路线作为默认值
  patrol_waypoints_ = {
      {"living_room", 0.0, -2.0, 0.0},
      {"doorway", 0.0, -1.0, 1.5708},
      {"hallway", 0.0, 1.5, 1.5708},
      {"bedroom", -2.5, 1.5, 0.0},
      {"hallway_return", 0.0, 1.5, 0.0},
      {"kitchen", 3.0, 2.0, 0.0},
      {"dock_approach", 0.0, -1.0, 1.5708},
      {"dock", 0.0, -2.0, 3.14159},
  };
  RCLCPP_INFO(this->get_logger(), "从配置加载巡逻路线: %zu 个航点",
              patrol_waypoints_.size());

  state_start_time_ = this->now();

  // 定时器：每 2 秒巡逻一次
  timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&PatrolMissionNode::timerCallback, this));

  RCLCPP_INFO(this->get_logger(), "%s", "==================================================");
  RCLCPP_INFO(this->get_logger(), "  Puppy 家庭巡逻系统启动");
  RCLCPP_INFO(this->get_logger(), "  巡逻路线: 客厅→走廊→卧室→厨房→充电桩");
  RCLCPP_INFO(this->get_logger(), "  低电量阈值: %.0f%%",
              low_battery_threshold_ * 100.0);
  RCLCPP_INFO(this->get_logger(), "%s", "==================================================");

  publishStatus("系统启动，准备巡逻");
}

void PatrolMissionNode::batteryCallback(
    const sensor_msgs::msg::BatteryState::SharedPtr msg) {
  // 电池状态回调
  battery_level_ = msg->percentage;
  if (msg->power_supply_status ==
      sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING) {
    is_charging_ = true;
  } else {
    is_charging_ = false;
  }
}

void PatrolMissionNode::fallCallback(
    const std_msgs::msg::Bool::SharedPtr msg) {
  // 跌倒检测回调
  if (msg->data && state_ != PatrolState::Emergency) {
    emergency_stop_ = true;
    state_ = PatrolState::Emergency;
    state_start_time_ = this->now();
    std_msgs::msg::String alert;
    alert.data = "跌倒检测警报！检测到老人跌倒！";
    alert_pub_->publish(alert);
    publishStatus("紧急：检测到跌倒！");
    RCLCPP_ERROR(this->get_logger(), "%s", alert.data.c_str());
  }
}

void PatrolMissionNode::emotionCallback(
    const std_msgs::msg::String::SharedPtr msg) {
  // 表情识别回调
  const std::string &emotion = msg->data;
  RCLCPP_INFO(this->get_logger(), "检测到情绪: %s", emotion.c_str());
  // 根据情绪播放相应回应
  std::string response;
  if (emotion == "happy") {
    response = "主人看起来很开心，我也很高兴！";
  } else if (emotion == "sad") {
    response = "主人看起来不开心，我来陪陪你吧";
  } else if (emotion == "angry") {
    response = "主人在生气，我安静一会儿";
  } else if (emotion == "surprised") {
    response = "主人惊讶了，发生什么事了？";
  } else if (emotion == "neutral") {
    response = "主人状态正常";
  } else {
    response = "主人在呢";
  }
  std_msgs::msg::String emotion_cmd;
  emotion_cmd.data = "speak:" + response;
  emotion_pub_->publish(emotion_cmd);
}

void PatrolMissionNode::publishStatus(const std::string &message) {
  // 发布状态
  std::string state_str;
  switch (state_) {
    case PatrolState::Idle:
      state_str = "IDLE";
      break;
    case PatrolState::Patrol:
      state_str = "PATROL";
      break;
    case PatrolState::ReturnCharge:
      state_str = "RETURN_CHARGE";
      break;
    case PatrolState::Charging:
      state_str = "CHARGING";
      break;
    case PatrolState::Emergency:
      state_str = "EMERGENCY";
      break;
  }
  std_msgs::msg::String status;
  status.data = "[" + state_str + "] " + message;
  status_pub_->publish(status);
  RCLCPP_INFO(this->get_logger(), "%s", status.data.c_str());
}

void PatrolMissionNode::timerCallback() {
  // 主状态机
  // 紧急状态处理
  if (state_ == PatrolState::Emergency) {
    handleEmergency();
    return;
  }

  // 低电量检查
  if (battery_level_ < low_battery_threshold_ &&
      state_ != PatrolState::ReturnCharge &&
      state_ != PatrolState::Charging) {
    state_ = PatrolState::ReturnCharge;
    state_start_time_ = this->now();
    char buf[64];
    std::snprintf(buf, sizeof(buf), "低电量(%.0f%%)，返回充电",
                  battery_level_ * 100.0);
    publishStatus(buf);
    navigateToChargingDock();
    return;
  }

  if (state_ == PatrolState::Idle) {
    // 开始巡逻
    state_ = PatrolState::Patrol;
    current_waypoint_ = 0;
    state_start_time_ = this->now();
    publishStatus("开始第 " + std::to_string(patrol_count_ + 1) + " 次巡逻");
    navigateToWaypoint(current_waypoint_);

  } else if (state_ == PatrolState::Patrol) {
    if (!is_navigating_) {
      // 到达当前路径点，扫描环境
      if (current_waypoint_ < patrol_waypoints_.size()) {
        const auto &wp = patrol_waypoints_[current_waypoint_];
        publishStatus("到达 " + wp.name + "，扫描环境...");
        scanEnvironment();

        current_waypoint_++;
        if (current_waypoint_ >= patrol_waypoints_.size()) {
          // 巡逻完成，返回充电
          patrol_count_++;
          state_ = PatrolState::ReturnCharge;
          publishStatus("巡逻完成(第" + std::to_string(patrol_count_) +
                        "次)，返回充电桩");
          navigateToChargingDock();
        } else {
          // 前往下一个路径点
          navigateToWaypoint(current_waypoint_);
        }
      }
    }

  } else if (state_ == PatrolState::ReturnCharge) {
    if (!is_navigating_) {
      state_ = PatrolState::Charging;
      state_start_time_ = this->now();
      publishStatus("已到达充电桩，开始充电");
      // 播放情绪交互
      std_msgs::msg::String emotion_cmd;
      emotion_cmd.data = "speak:巡逻完成，我回来充电啦，主人辛苦了";
      emotion_pub_->publish(emotion_cmd);
    }

  } else if (state_ == PatrolState::Charging) {
    // 模拟充电过程
    double elapsed = (this->now() - state_start_time_).seconds();
    if (elapsed > 10.0) {  // 充电 10 秒后继续巡逻
      battery_level_ = 1.0;
      state_ = PatrolState::Idle;
      publishStatus("充电完成，准备下一次巡逻");
    }
  }
}

void PatrolMissionNode::navigateToWaypoint(size_t index) {
  // 导航到路径点
  if (index >= patrol_waypoints_.size()) {
    return;
  }
  const auto &wp = patrol_waypoints_[index];
  char buf[128];
  std::snprintf(buf, sizeof(buf), "导航到 %s (%.1f, %.1f)", wp.name.c_str(),
                wp.x, wp.y);
  publishStatus(buf);
  sendNavigationGoal(wp.x, wp.y, wp.yaw);
}

void PatrolMissionNode::navigateToChargingDock() {
  // 返回充电桩
  publishStatus("导航返回充电桩");
  sendNavigationGoal(charging_dock_.x, charging_dock_.y,
                     charging_dock_.yaw);
}

void PatrolMissionNode::sendNavigationGoal(double x, double y, double yaw) {
  // 发送导航目标
  if (!nav_client_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_WARN(this->get_logger(), "导航服务器不可用，使用直接速度控制");
    directNavigate(x, y, yaw);
    return;
  }

  NavigateToPose::Goal goal_msg;
  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.header.stamp = this->now();
  goal_msg.pose.pose.position.x = x;
  goal_msg.pose.pose.position.y = y;
  goal_msg.pose.pose.position.z = 0.0;

  // Yaw 转四元数
  double qz = std::sin(yaw / 2.0);
  double qw = std::cos(yaw / 2.0);
  goal_msg.pose.pose.orientation.x = 0.0;
  goal_msg.pose.pose.orientation.y = 0.0;
  goal_msg.pose.pose.orientation.z = qz;
  goal_msg.pose.pose.orientation.w = qw;

  is_navigating_ = true;

  auto send_goal_options =
      rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback = std::bind(
      &PatrolMissionNode::goalResponseCallback, this, std::placeholders::_1);
  send_goal_options.result_callback =
      std::bind(&PatrolMissionNode::resultCallback, this,
                std::placeholders::_1);

  nav_client_->async_send_goal(goal_msg, send_goal_options);
}

void PatrolMissionNode::goalResponseCallback(
    std::shared_future<rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr>
    future) {
  // 导航目标响应
  auto goal_handle = future.get();
  if (!goal_handle) {
    RCLCPP_WARN(this->get_logger(), "导航目标被拒绝");
    is_navigating_ = false;
    return;
  }
  // 结果由 result_callback 处理
}

void PatrolMissionNode::resultCallback(
    const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult &
        result) {
  // 导航结果
  is_navigating_ = false;
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(this->get_logger(), "导航完成");
  } else {
    RCLCPP_WARN(this->get_logger(), "导航失败");
  }
}

void PatrolMissionNode::directNavigate(double /*x*/, double /*y*/,
                                       double /*yaw*/) {
  // 直接速度控制导航（备用方案）
  is_navigating_ = true;
  // 简单的前进+旋转
  geometry_msgs::msg::Twist twist;
  twist.linear.x = 0.2;
  twist.angular.z = 0.0;
  double duration = 3.0;  // 每个点走 3 秒

  auto start = std::chrono::steady_clock::now();
  while (true) {
    auto elapsed = std::chrono::duration<double>(
                       std::chrono::steady_clock::now() - start)
                       .count();
    if (elapsed >= duration || !rclcpp::ok()) {
      break;
    }
    cmd_pub_->publish(twist);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  geometry_msgs::msg::Twist stop;
  cmd_pub_->publish(stop);
  is_navigating_ = false;
}

void PatrolMissionNode::scanEnvironment() {
  // 扫描环境（原地旋转）
  geometry_msgs::msg::Twist twist;
  twist.angular.z = 0.5;  // 旋转速度
  double duration = 4.0;  // 旋转 4 秒

  auto start = std::chrono::steady_clock::now();
  while (true) {
    auto elapsed = std::chrono::duration<double>(
                       std::chrono::steady_clock::now() - start)
                       .count();
    if (elapsed >= duration || !rclcpp::ok()) {
      break;
    }
    cmd_pub_->publish(twist);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  geometry_msgs::msg::Twist stop;
  cmd_pub_->publish(stop);
}

void PatrolMissionNode::handleEmergency() {
  // 紧急状态处理
  double elapsed = (this->now() - state_start_time_).seconds();
  // 紧急状态持续 30 秒后恢复正常
  if (elapsed > 30.0) {
    emergency_stop_ = false;
    state_ = PatrolState::ReturnCharge;
    publishStatus("紧急状态解除，返回充电");
    navigateToChargingDock();
  }
}

}  // namespace puppy_gait
