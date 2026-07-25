// all_gait_nodes.h: puppy_gait C++ 节点汇总头文件
//
// 引入此头即可使用 puppy_gait 命名空间下的全部 ROS2 节点类。
// 对应 Python 包 src/puppy_gait/scripts/ 下的节点实现。
//
// 转换说明：
//   所有节点从 Python (rclpy) 转换为 C++ (rclcpp)，
//   使用 rclcpp::Node 替代 rclpy.Node，
//   使用 publisher_/subscribe_ 模式替代 Python 的 create_publisher/create_subscription，
//   使用 timer_ 替代 create_timer。
//   numpy 操作已用 std::vector / 原生数学替代（本组节点未使用 numpy）。
#pragma once

// 自动巡航节点
#include "puppy_gait/auto_cruise.h"

// 老人跌倒检测节点
#include "puppy_gait/fall_detection.h"

// 电池仿真节点
#include "puppy_gait/battery_simulator.h"

// 情绪交互节点
#include "puppy_gait/emotion_interaction.h"

// 家庭安防节点
#include "puppy_gait/security_node.h"

// 智能家居家电管理节点
#include "puppy_gait/smart_home_manager.h"

// 定时任务调度节点
#include "puppy_gait/task_scheduler.h"

// 家庭巡逻任务节点（legacy 状态机）
#include "puppy_gait/patrol_mission.h"

// 紧急联动响应节点
#include "puppy_gait/emergency_response.h"

namespace puppy_gait {

/// puppy_gait C++ 节点集合（便于在 main 中统一注册 / 列举）
///
/// 用法示例:
/// @code
///   rclcpp::init(argc, argv);
///   rclcpp::executors::MultiThreadedExecutor exec;
///   exec.add_node(std::make_shared<puppy_gait::AutoCruiseNode>());
///   exec.add_node(std::make_shared<puppy_gait::SecurityNode>());
///   ...
///   exec.spin();
/// @endcode
struct AllGaitNodes {
  // 类型别名，方便上层引用
  using AutoCruise         = AutoCruiseNode;
  using FallDetection      = FallDetectionNode;
  using BatterySimulator   = BatterySimulatorNode;
  using EmotionInteraction = EmotionInteractionNode;
  using Security           = SecurityNode;
  using SmartHomeManager   = SmartHomeManagerNode;
  using TaskScheduler      = TaskSchedulerNode;
  using PatrolMission      = PatrolMissionNode;
  using EmergencyResponse  = EmergencyResponseNode;
};

}  // namespace puppy_gait
