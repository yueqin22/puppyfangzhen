// Puppy 机械狗 - 智能家庭巡逻系统（C++ legacy 状态机）
//
// RUNTIME: demo (legacy, superseded by mission_manager as authoritative orchestrator)
// 权威编排层: puppy_core/mission_manager (IDLE/PATROL/DOCKING/NAVIGATION)
// 本节点保留独立状态机（IDLE/PATROL/RETURN_CHARGE/CHARGING/EMERGENCY）用于
// 演示和对照测试，不进入产品化 bringup。正式巡逻请通过 mission_manager 触发。
//
// 功能：
//   1. 自动巡逻（客厅→走廊→卧室→厨房→客厅）
//   2. 巡逻中检测异常（跌倒、入侵）
//   3. 巡逻完成后返回充电桩
//   4. 低电量自动回充
//   5. 情绪交互（播放语音、表情回应）
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

namespace puppy_gait {

/// 巡逻路径点
struct Waypoint {
  std::string name;
  double x;
  double y;
  double yaw;
};

/// 巡逻状态机枚举
enum class PatrolState {
  Idle,
  Patrol,
  ReturnCharge,
  Charging,
  Emergency,
};

/// 家庭巡逻任务节点
class PatrolMissionNode : public rclcpp::Node {
 public:
  PatrolMissionNode();
  ~PatrolMissionNode() override = default;

 private:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;

  // ===== 回调 =====
  void batteryCallback(const sensor_msgs::msg::BatteryState::SharedPtr msg);
  void fallCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void emotionCallback(const std_msgs::msg::String::SharedPtr msg);

  // 主状态机
  void timerCallback();

  // 发布状态
  void publishStatus(const std::string &message);

  // 导航到路径点
  void navigateToWaypoint(size_t index);
  // 返回充电桩
  void navigateToChargingDock();
  // 发送导航目标
  void sendNavigationGoal(double x, double y, double yaw);
  // 直接速度控制导航（备用方案）
  void directNavigate(double x, double y, double yaw);
  // 扫描环境（原地旋转）
  void scanEnvironment();
  // 紧急状态处理
  void handleEmergency();

  // Action 回调
  void goalResponseCallback(
      std::shared_future<rclcpp_action::ClientGoalHandle<NavigateToPose>::SharedPtr>
          future);
  void resultCallback(
      const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult &
          result);

 private:
  // 导航客户端
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;

  // 发布器
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr alert_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr emotion_pub_;

  // 订阅器
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fall_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr emotion_sub_;

  // 定时器
  rclcpp::TimerBase::SharedPtr timer_;

  // 状态
  double battery_level_{1.0};       // 电量 0-1
  double low_battery_threshold_{0.2};
  bool is_charging_{false};
  int patrol_count_{0};
  size_t current_waypoint_{0};
  bool is_navigating_{false};
  bool emergency_stop_{false};
  PatrolState state_{PatrolState::Idle};
  rclcpp::Time state_start_time_;

  // 充电桩位置（起点）— 对应 navigation_targets.yaml 中的 dock
  Waypoint charging_dock_{"dock", 0.0, -2.0, 0.0};

  // P0-5: 巡逻路径点（从 patrol_routes.yaml 加载，回退到硬编码）
  std::vector<Waypoint> patrol_waypoints_;
};

}  // namespace puppy_gait
