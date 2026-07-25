// Config Utils: 加载共享运行时配置的工具函数 (C++ 版本)
//
// 纯工具函数（非 ROS2 节点），对应 Python: puppy_core/config_utils.py
// 依赖：yaml-cpp、ament_index_cpp
#pragma once

#include <map>
#include <string>
#include <vector>

namespace puppy_core {

/// 导航目标点（对应 Python 中 {x, y, yaw} 字典）
struct NavigationTarget {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};

  NavigationTarget() = default;
  NavigationTarget(double x_, double y_, double yaw_) : x(x_), y(y_), yaw(yaw_) {}
};

/// 从 puppy_core/config/<filename> 加载 YAML 并解析
/// 对应 Python: _load_yaml
/// 返回是否加载成功；解析结果通过 yaml-cpp Node 返回（需要 yaml-cpp 头文件）
/// 为避免在头文件中暴露 yaml-cpp，此处仅声明，实现在 .cpp 中。

/// 加载命名导航目标集合
/// 对应 Python: load_navigation_targets
/// 返回 {name: NavigationTarget} 映射
std::map<std::string, NavigationTarget> loadNavigationTargets();

/// 获取指定名称的导航目标
/// 对应 Python: get_named_target
/// 找不到时返回 false，并通过 out 输出
bool getNamedTarget(const std::string& name, NavigationTarget& out);

/// 获取配置的充电桩目标
/// 对应 Python: get_dock_target
bool getDockTarget(NavigationTarget& out);

/// 加载命名巡逻路线集合
/// 对应 Python: load_patrol_routes
/// 返回 {route_name: [waypoint, ...]} 映射
std::map<std::string, std::vector<NavigationTarget>> loadPatrolRoutes();

/// 获取指定名称的巡逻路线
/// 对应 Python: get_patrol_route
/// 找不到时返回 default_route
std::vector<NavigationTarget> getPatrolRoute(
    const std::string& name = "default",
    const std::vector<NavigationTarget>& default_route = {});

}  // namespace puppy_core
