// Goal Dispatcher 实现
#include "puppy_core/goal_dispatcher.h"

#include <algorithm>
#include <cmath>

namespace puppy_core {

GoalDispatcherNode::GoalDispatcherNode()
    : rclcpp::Node("goal_dispatcher") {
  nav_client_ = rclcpp_action::create_client<NavigateToPose>(
      this, "navigate_to_pose");

  named_goal_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/goal/named", 10,
      std::bind(&GoalDispatcherNode::onNamedGoal, this,
                std::placeholders::_1));
  pose_goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal/pose", 10,
      std::bind(&GoalDispatcherNode::onPoseGoal, this,
                std::placeholders::_1));

  // C++版未接入YAML配置加载，默认命名目标为空
  // 若后续接入 config_utils，应在此调用 load_navigation_targets()
  RCLCPP_INFO(this->get_logger(),
              "Goal dispatcher started with %zu named goals",
              named_goals_.size());
}

void GoalDispatcherNode::onNamedGoal(
    const std_msgs::msg::String::SharedPtr msg) {
  // Handle named goal request.
  std::string name = msg->data;
  std::transform(name.begin(), name.end(), name.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  auto it = named_goals_.find(name);
  if (it == named_goals_.end()) {
    RCLCPP_WARN(this->get_logger(), "Unknown named goal: %s", name.c_str());
    return;
  }
  const NamedGoal& goal = it->second;
  sendGoal(goal.x, goal.y, goal.yaw);
}

void GoalDispatcherNode::onPoseGoal(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  // Handle direct pose goal.
  std::string frame_id = msg->header.frame_id.empty()
                             ? std::string("map")
                             : msg->header.frame_id;
  sendGoal(msg->pose.position.x, msg->pose.position.y, 0.0, frame_id);
}

void GoalDispatcherNode::sendGoal(double x, double y, double yaw,
                                  const std::string& frame_id) {
  // Send goal to navigation action server.
  if (!nav_client_->wait_for_action_server(std::chrono::seconds(2))) {
    RCLCPP_ERROR(this->get_logger(), "Navigation action not available");
    return;
  }
  NavigateToPose::Goal goal;
  goal.pose.header.frame_id = frame_id;
  goal.pose.header.stamp = this->now();
  goal.pose.pose.position.x = x;
  goal.pose.pose.position.y = y;
  goal.pose.pose.orientation.z = std::sin(yaw / 2.0);
  goal.pose.pose.orientation.w = std::cos(yaw / 2.0);
  nav_client_->async_send_goal(goal);
  RCLCPP_INFO(this->get_logger(),
              "Goal sent: (%.1f, %.1f, yaw=%.2f)", x, y, yaw);
}

}  // namespace puppy_core
