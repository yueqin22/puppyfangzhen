// Shared navigation and patrol config for simulation-side helper scripts (C++ 版本)
//
// 工具类（非 ROS2 节点），对应 Python: puppy_gait/scripts/shared_targets.py
// 依赖：yaml-cpp、ament_index_cpp
//
// 注意：与 puppy_core/config_utils.h 功能等价（同一份配置），
// 这里保留独立的 puppy_gait 命名空间版本，方便 puppy_gait 内部脚本调用。
#pragma once

#include <map>
#include <string>
#include <vector>

namespace puppy_gait {

/// 导航目标点（对应 Python 中 {x, y, yaw} 字典）
struct NavigationTarget {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};

  NavigationTarget() = default;
  NavigationTarget(double x_, double y_, double yaw_) : x(x_), y(y_), yaw(yaw_) {}
};

/// 从 puppy_core/config/<filename> 加载 YAML 并返回命名导航目标集合
/// 对应 Python: load_navigation_targets
std::map<std::string, NavigationTarget> loadNavigationTargets();

/// 获取指定名称的导航目标
/// 对应 Python: get_named_target
/// 找不到时返回 false，并通过 out 输出
bool getNamedTarget(const std::string& name, NavigationTarget& out);

/// 加载命名巡逻路线集合
/// 对应 Python: load_patrol_routes
std::map<std::string, std::vector<NavigationTarget>> loadPatrolRoutes();

/// 获取指定名称的巡逻路线
/// 对应 Python: get_patrol_route
/// 找不到时返回 default_route
std::vector<NavigationTarget> getPatrolRoute(
    const std::string& name = "default",
    const std::vector<NavigationTarget>& default_route = {});

}  // namespace puppy_gait
