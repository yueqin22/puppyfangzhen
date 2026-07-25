// Mission Manager: high-level task orchestration over Nav2 primitives.
//
// 通过 NavigateToPose action 编排高层任务（巡逻、回充、手动导航等）。
#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "std_msgs/msg/string.hpp"
#include "puppy_interfaces/msg/battery_status.hpp"
#include "puppy_interfaces/msg/patrol_status.hpp"
#include "puppy_interfaces/msg/security_event.hpp"

namespace puppy_core {

/// 巡航航点
struct Waypoint {
  std::string name;
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

/// Orchestrate high-level robot missions.
class MissionManagerNode : public rclcpp::Node {
 public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose =
      rclcpp_action::ClientGoalHandle<NavigateToPose>;

  MissionManagerNode();
  ~MissionManagerNode() override = default;

 private:
  // 能力状态字符串处理（C++版简化：默认视为可用，等价Python cap is None）
  void onCapabilities(const std_msgs::msg::String::SharedPtr msg);
  bool isCapabilityAvailable(const std::string& name) const;

  void onCommand(const std_msgs::msg::String::SharedPtr msg);
  void startManualNavigation(const std::string& cmd);

  void onSecurity(const puppy_interfaces::msg::SecurityEvent::SharedPtr msg);
  void onBattery(const puppy_interfaces::msg::BatteryStatus::SharedPtr msg);

  // 启动一次导航；on_complete 在结果返回后被调用
  using CompletionCallback = std::function<void(bool success)>;
  bool startNavigation(double x, double y, double yaw,
                       CompletionCallback on_complete);

  void startPatrol();
  void startDocking();
  void cancelMission();
  void dispatchPatrolWaypoint();

  // Action 回调
  void onNavGoalResponse(
      std::shared_future<GoalHandleNavigateToPose::SharedPtr> future);
  void onNavResult(
      const GoalHandleNavigateToPose::WrappedResult& result);

  void handleNavCompletion(bool success);

  // 完成回调实现
  void onPatrolWaypointResult(bool success);
  void onDockResult(bool success);
  void onGotoResult(bool success);

  void publishStatus();

 private:
  std::string current_mission_{"IDLE"};
  Waypoint dock_target_;
  std::vector<Waypoint> patrol_route_;
  int current_waypoint_index_{-1};
  std::shared_ptr<GoalHandleNavigateToPose> goal_handle_;
  CompletionCallback pending_after_nav_;
  std::string last_status_message_{"idle"};
  // 能力状态原始JSON字符串（C++版不解析，保守起见视为可用）
  std::string capabilities_json_;

  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Subscription<puppy_interfaces::msg::SecurityEvent>::SharedPtr
      security_sub_;
  rclcpp::Subscription<puppy_interfaces::msg::BatteryStatus>::SharedPtr
      battery_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr capabilities_sub_;

  rclcpp::Publisher<puppy_interfaces::msg::PatrolStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace puppy_core
