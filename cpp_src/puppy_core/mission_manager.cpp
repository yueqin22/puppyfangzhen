// Mission Manager 实现
#include "puppy_core/mission_manager.h"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace puppy_core {

MissionManagerNode::MissionManagerNode()
    : rclcpp::Node("mission_manager") {
  // 默认 dock 目标（对应 Python: get_dock_target({'x':0.0,'y':-2.0,'yaw':0.0})）
  dock_target_.name = "dock";
  dock_target_.x = 0.0;
  dock_target_.y = -2.0;
  dock_target_.yaw = 0.0;

  // 默认巡逻路线（对应 Python: get_patrol_route() 在无配置时返回空列表）
  // 当配置加载不可用时使用空列表，保持与Python一致的"no patrol route"行为。

  nav_client_ = rclcpp_action::create_client<NavigateToPose>(
      this, "navigate_to_pose");

  command_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/mission/command", 10,
      std::bind(&MissionManagerNode::onCommand, this, std::placeholders::_1));
  security_sub_ =
      this->create_subscription<puppy_interfaces::msg::SecurityEvent>(
          "/security/event", 10,
          std::bind(&MissionManagerNode::onSecurity, this,
                    std::placeholders::_1));
  battery_sub_ =
      this->create_subscription<puppy_interfaces::msg::BatteryStatus>(
          "/battery_status", 10,
          std::bind(&MissionManagerNode::onBattery, this,
                    std::placeholders::_1));
  capabilities_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/robot/capabilities", 10,
      std::bind(&MissionManagerNode::onCapabilities, this,
                std::placeholders::_1));

  status_pub_ = this->create_publisher<puppy_interfaces::msg::PatrolStatus>(
      "/mission/status", 10);
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&MissionManagerNode::publishStatus, this));

  RCLCPP_INFO(this->get_logger(),
              "Mission manager started with patrol route of %zu waypoints",
              patrol_route_.size());
}

void MissionManagerNode::onCapabilities(
    const std_msgs::msg::String::SharedPtr msg) {
  // 保存原始字符串，C++版默认将所有能力视为可用
  // （Python 中 cap is None 时返回 True）
  capabilities_json_ = msg->data;
}

bool MissionManagerNode::isCapabilityAvailable(
    const std::string& /*name*/) const {
  // 简化处理：不解析JSON，保守地将所有能力视为可用
  // 若后续接入完整 JSON 解析，应检查 enabled && available
  return true;
}

void MissionManagerNode::onCommand(
    const std_msgs::msg::String::SharedPtr msg) {
  std::string cmd = msg->data;
  // 转小写
  std::transform(cmd.begin(), cmd.end(), cmd.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  RCLCPP_INFO(this->get_logger(), "Mission command: %s", cmd.c_str());
  if (cmd == "patrol") {
    startPatrol();
  } else if (cmd == "dock") {
    startDocking();
  } else if (cmd == "stop") {
    cancelMission();
  } else if (cmd.rfind("goto:", 0) == 0) {
    startManualNavigation(cmd);
  }
}

void MissionManagerNode::startManualNavigation(const std::string& cmd) {
  if (!isCapabilityAvailable("navigation")) {
    RCLCPP_WARN(this->get_logger(),
                "Navigation capability unavailable - manual goal rejected");
    last_status_message_ = "manual goal rejected: navigation unavailable";
    return;
  }

  // 解析 "goto:x,y"
  double goal_x = 0.0;
  double goal_y = 0.0;
  try {
    auto colon_pos = cmd.find(':');
    if (colon_pos == std::string::npos) {
      throw std::runtime_error("expected x,y");
    }
    std::string coords_str = cmd.substr(colon_pos + 1);
    auto comma_pos = coords_str.find(',');
    if (comma_pos == std::string::npos) {
      throw std::runtime_error("expected x,y");
    }
    std::string x_str = coords_str.substr(0, comma_pos);
    std::string y_str = coords_str.substr(comma_pos + 1);
    // 去除空白
    x_str.erase(std::remove(x_str.begin(), x_str.end(), ' '), x_str.end());
    y_str.erase(std::remove(y_str.begin(), y_str.end(), ' '), y_str.end());
    if (x_str.empty() || y_str.empty()) {
      throw std::runtime_error("expected x,y");
    }
    goal_x = std::stod(x_str);
    goal_y = std::stod(y_str);
  } catch (const std::exception& exc) {
    RCLCPP_WARN(this->get_logger(), "Invalid goto command \"%s\": %s",
                cmd.c_str(), exc.what());
    last_status_message_ = "manual goal rejected: invalid coordinates";
    return;
  }

  if (!nav_client_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_ERROR(this->get_logger(), "NavigateToPose action not available");
    last_status_message_ =
        "manual goal rejected: navigation action unavailable";
    return;
  }

  cancelMission();
  bool started = startNavigation(
      goal_x, goal_y, 0.0,
      [this](bool success) { this->onGotoResult(success); });
  if (started) {
    current_mission_ = "NAVIGATION";
    std::ostringstream oss;
    oss << "manual goal (" << goal_x << ", " << goal_y << ")";
    last_status_message_ = oss.str();
  }
}

void MissionManagerNode::onSecurity(
    const puppy_interfaces::msg::SecurityEvent::SharedPtr msg) {
  if (msg->severity == "critical") {
    RCLCPP_WARN(this->get_logger(),
                "Security alert: %s - interrupting mission",
                msg->event_type.c_str());
    cancelMission();
  }
}

void MissionManagerNode::onBattery(
    const puppy_interfaces::msg::BatteryStatus::SharedPtr msg) {
  if (msg->low_battery && current_mission_ == "PATROL") {
    RCLCPP_INFO(this->get_logger(), "Low battery - returning to dock");
    cancelMission();
    startDocking();
  }
}

bool MissionManagerNode::startNavigation(double x, double y, double yaw,
                                         CompletionCallback on_complete) {
  if (!nav_client_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_ERROR(this->get_logger(), "NavigateToPose action not available");
    last_status_message_ = "navigation action unavailable";
    return false;
  }

  NavigateToPose::Goal goal;
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = this->now();
  goal.pose.pose.position.x = x;
  goal.pose.pose.position.y = y;
  goal.pose.pose.orientation.z = std::sin(yaw / 2.0);
  goal.pose.pose.orientation.w = std::cos(yaw / 2.0);

  pending_after_nav_ = std::move(on_complete);

  auto send_goal_options =
      rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      std::bind(&MissionManagerNode::onNavGoalResponse, this,
                std::placeholders::_1);
  send_goal_options.result_callback =
      std::bind(&MissionManagerNode::onNavResult, this,
                std::placeholders::_1);
  nav_client_->async_send_goal(goal, send_goal_options);
  return true;
}

void MissionManagerNode::startPatrol() {
  if (patrol_route_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "No patrol route configured");
    last_status_message_ = "no patrol route configured";
    return;
  }
  if (!isCapabilityAvailable("patrol")) {
    RCLCPP_WARN(this->get_logger(),
                "Patrol capability unavailable - patrol rejected");
    current_mission_ = "IDLE";
    last_status_message_ = "patrol rejected: capability unavailable";
    return;
  }
  if (!isCapabilityAvailable("navigation")) {
    RCLCPP_WARN(this->get_logger(),
                "Navigation capability unavailable - patrol rejected");
    current_mission_ = "IDLE";
    last_status_message_ = "patrol rejected: navigation unavailable";
    return;
  }

  cancelMission();
  current_mission_ = "PATROL";
  current_waypoint_index_ = 0;
  last_status_message_ = "patrol started";
  dispatchPatrolWaypoint();
}

void MissionManagerNode::startDocking() {
  if (!isCapabilityAvailable("docking")) {
    RCLCPP_WARN(this->get_logger(),
                "Docking capability unavailable - docking rejected");
    current_mission_ = "IDLE";
    last_status_message_ = "docking rejected: capability unavailable";
    return;
  }
  if (!isCapabilityAvailable("navigation")) {
    RCLCPP_WARN(this->get_logger(),
                "Navigation capability unavailable - docking rejected");
    current_mission_ = "IDLE";
    last_status_message_ = "docking rejected: navigation unavailable";
    return;
  }

  cancelMission();
  current_mission_ = "DOCKING";
  current_waypoint_index_ = -1;
  last_status_message_ = "returning to dock";
  bool started = startNavigation(
      dock_target_.x, dock_target_.y, dock_target_.yaw,
      [this](bool success) { this->onDockResult(success); });
  if (!started) {
    current_mission_ = "IDLE";
  }
}

void MissionManagerNode::cancelMission() {
  if (goal_handle_) {
    try {
      nav_client_->async_cancel_goal(goal_handle_);
    } catch (const std::exception& exc) {
      RCLCPP_WARN(this->get_logger(),
                  "Failed to cancel navigation goal: %s", exc.what());
    }
  }
  goal_handle_.reset();
  pending_after_nav_ = nullptr;
  current_waypoint_index_ = -1;
  current_mission_ = "IDLE";
  last_status_message_ = "mission cancelled";
  RCLCPP_INFO(this->get_logger(), "Mission cancelled");
}

void MissionManagerNode::dispatchPatrolWaypoint() {
  if (current_mission_ != "PATROL") {
    return;
  }

  if (current_waypoint_index_ >= static_cast<int>(patrol_route_.size())) {
    RCLCPP_INFO(this->get_logger(),
                "Patrol route complete - returning to dock");
    startDocking();
    return;
  }

  const Waypoint& waypoint = patrol_route_[current_waypoint_index_];
  std::ostringstream name_oss;
  if (waypoint.name.empty()) {
    name_oss << current_waypoint_index_;
  } else {
    name_oss << waypoint.name;
  }
  last_status_message_ = "navigating to " + name_oss.str();
  RCLCPP_INFO(this->get_logger(),
              "Patrol waypoint %d/%zu: %s (%.2f, %.2f)",
              current_waypoint_index_ + 1, patrol_route_.size(),
              waypoint.name.empty() ? "waypoint" : waypoint.name.c_str(),
              waypoint.x, waypoint.y);
  bool started = startNavigation(
      waypoint.x, waypoint.y, waypoint.yaw,
      [this](bool success) { this->onPatrolWaypointResult(success); });
  if (!started) {
    current_mission_ = "IDLE";
  }
}

void MissionManagerNode::onNavGoalResponse(
    std::shared_future<GoalHandleNavigateToPose::SharedPtr> future) {
  GoalHandleNavigateToPose::SharedPtr goal_handle = future.get();
  if (!goal_handle || !goal_handle->is_active()) {
    // 等价 Python: not goal_handle.accepted
    goal_handle_.reset();
    RCLCPP_ERROR(this->get_logger(), "Navigation goal rejected");
    last_status_message_ = "navigation goal rejected";
    handleNavCompletion(false);
    return;
  }
  goal_handle_ = goal_handle;
}

void MissionManagerNode::onNavResult(
    const GoalHandleNavigateToPose::WrappedResult& result) {
  goal_handle_.reset();
  bool success = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
  if (success) {
    last_status_message_ = "navigation goal reached";
  } else {
    std::ostringstream oss;
    oss << "navigation failed (code=" << static_cast<int>(result.code) << ")";
    last_status_message_ = oss.str();
    RCLCPP_WARN(this->get_logger(), "%s", last_status_message_.c_str());
  }
  handleNavCompletion(success);
}

void MissionManagerNode::handleNavCompletion(bool success) {
  CompletionCallback callback = pending_after_nav_;
  pending_after_nav_ = nullptr;
  if (callback) {
    callback(success);
  }
}

void MissionManagerNode::onPatrolWaypointResult(bool success) {
  if (current_mission_ != "PATROL") {
    return;
  }

  if (!success) {
    const Waypoint& failed = patrol_route_[current_waypoint_index_];
    std::ostringstream name_oss;
    name_oss << (failed.name.empty()
                     ? std::to_string(current_waypoint_index_)
                     : failed.name);
    RCLCPP_WARN(this->get_logger(),
                "Patrol failed at %s; aborting patrol",
                name_oss.str().c_str());
    current_mission_ = "IDLE";
    last_status_message_ = "patrol aborted";
    return;
  }

  const Waypoint& reached = patrol_route_[current_waypoint_index_];
  std::ostringstream name_oss;
  name_oss << (reached.name.empty()
                   ? std::to_string(current_waypoint_index_)
                   : reached.name);
  last_status_message_ = "reached " + name_oss.str();
  current_waypoint_index_ += 1;
  dispatchPatrolWaypoint();
}

void MissionManagerNode::onDockResult(bool success) {
  current_waypoint_index_ = -1;
  current_mission_ = "IDLE";
  if (success) {
    last_status_message_ = "docked";
    RCLCPP_INFO(this->get_logger(), "Dock target reached");
  } else {
    last_status_message_ = "dock navigation failed";
    RCLCPP_WARN(this->get_logger(), "Failed to reach dock target");
  }
}

void MissionManagerNode::onGotoResult(bool success) {
  current_mission_ = "IDLE";
  last_status_message_ = success ? "manual goal reached" : "manual goal failed";
}

void MissionManagerNode::publishStatus() {
  puppy_interfaces::msg::PatrolStatus msg;
  msg.header.stamp = this->now();
  msg.state = current_mission_;
  msg.current_waypoint_index = current_waypoint_index_;
  msg.total_waypoints = static_cast<int32_t>(patrol_route_.size());
  if (current_mission_ == "PATROL" && !patrol_route_.empty()) {
    double ratio = static_cast<double>(current_waypoint_index_) /
                   static_cast<double>(patrol_route_.size());
    msg.completion_ratio = static_cast<float>(std::max(0.0, std::min(1.0, ratio)));
  } else if (current_mission_ == "DOCKING") {
    msg.completion_ratio = 1.0f;
  } else {
    msg.completion_ratio = 0.0f;
  }
  msg.message = last_status_message_;
  status_pub_->publish(msg);
}

}  // namespace puppy_core
