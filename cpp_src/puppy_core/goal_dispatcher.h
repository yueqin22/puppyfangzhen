// Goal Dispatcher: routes named goals to the active navigation stack.
//
// 将命名目标 / 直接位姿目标路由到活动的导航栈 (NavigateToPose action)。
#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

namespace puppy_core {

/// 命名目标条目
struct NamedGoal {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

/// Dispatches navigation goals to the active planner.
class GoalDispatcherNode : public rclcpp::Node {
 public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;

  GoalDispatcherNode();
  ~GoalDispatcherNode() override = default;

 private:
  // Handle named goal request.
  void onNamedGoal(const std_msgs::msg::String::SharedPtr msg);
  // Handle direct pose goal.
  void onPoseGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  // Send goal to navigation action server.
  void sendGoal(double x, double y, double yaw,
                const std::string& frame_id = "map");

 private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr named_goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      pose_goal_sub_;

  // 命名目标表（C++版默认为空，等价 Python load_navigation_targets() 无配置时）
  std::unordered_map<std::string, NamedGoal> named_goals_;
};

}  // namespace puppy_core
