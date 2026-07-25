// all_nodes.h: puppy_core C++ 节点汇总头文件
//
// 引入此头即可使用 puppy_core 命名空间下的全部 ROS2 节点类。
// 对应 Python 包 src/puppy_core/puppy_core/ 下的节点实现。
//
// 注意：用户原始任务列表中的 dog_controller / dog_state / dog_motion_controller /
// dog_status_manager / dog_safety_monitor / dog_task_manager / dog_behavior_planner /
// dog_perception_system / dog_command_interface 等文件在源代码树中并不存在，
// 因此未在此汇总。若后续新增这些节点，请在对应位置补充 #include 并登记。
#pragma once

// 核心导航包装
#include "puppy_core/nav_core_node.h"

// 系统能力注册表
#include "puppy_core/capability_registry_node.h"

// 看门狗与故障恢复
#include "puppy_core/watchdog.h"

// 安全管理
#include "puppy_core/safety_manager.h"

// 模式管理（状态机）
#include "puppy_core/mode_manager.h"

// 任务编排（巡逻/回充/手动导航）
#include "puppy_core/mission_manager.h"

// 目标分发
#include "puppy_core/goal_dispatcher.h"

// 机器人状态聚合
#include "puppy_core/robot_state_aggregator.h"

// 电池状态适配
#include "puppy_core/battery_status_adapter.h"

// 跌倒事件适配
#include "puppy_core/fall_event_adapter.h"

namespace puppy_core {

/// puppy_core C++ 节点集合（便于在 main 中统一注册 / 列举）
///
/// 用法示例:
/// @code
///   rclcpp::init(argc, argv);
///   rclcpp::executors::MultiThreadedExecutor exec;
///   exec.add_node(std::make_shared<puppy_core::WatchdogNode>());
///   exec.add_node(std::make_shared<puppy_core::SafetyManagerNode>());
///   ...
///   exec.spin();
/// @endcode
struct AllNodes {
  // 类型别名，方便上层引用
  using NavCore               = NavCoreNode;
  using CapabilityRegistry    = CapabilityRegistryNode;
  using Watchdog              = WatchdogNode;
  using SafetyManager         = SafetyManagerNode;
  using ModeManager           = ModeManagerNode;
  using MissionManager        = MissionManagerNode;
  using GoalDispatcher        = GoalDispatcherNode;
  using RobotStateAggregator  = RobotStateAggregatorNode;
  using BatteryStatusAdapter  = BatteryStatusAdapterNode;
  using FallEventAdapter      = FallEventAdapterNode;
};

}  // namespace puppy_core
