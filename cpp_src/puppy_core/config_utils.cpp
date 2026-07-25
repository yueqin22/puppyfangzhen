// Config Utils 实现
//
// 加载共享运行时配置的工具函数
#include "puppy_core/config_utils.h"

#include <fstream>
#include <sstream>
#include <stdexcept>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

namespace puppy_core {

// 内部辅助：从 puppy_core/config/<filename> 加载 YAML 文件
// 对应 Python: _load_yaml
static YAML::Node loadYaml(const std::string& filename) {
  std::string pkg_share =
      ament_index_cpp::get_package_share_directory("puppy_core_cpp");
  std::string path = pkg_share + "/config/" + filename;
  std::ifstream fin(path);
  if (!fin.is_open()) {
    RCLCPP_ERROR(rclcpp::get_logger("config_utils"),
                 "无法打开配置文件: %s", path.c_str());
    return YAML::Node();  // 空 Node，等价 Python 的 {}
  }
  try {
    return YAML::Load(fin);
  } catch (const YAML::Exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("config_utils"),
                 "YAML 解析失败 (%s): %s", path.c_str(), e.what());
    return YAML::Node();
  }
}

// 内部辅助：从 YAML 节点解析 NavigationTarget
// 期望节点是包含 x/y/yaw 的 map
static NavigationTarget parseTarget(const YAML::Node& node) {
  NavigationTarget t;
  if (node["x"]) t.x = node["x"].as<double>();
  if (node["y"]) t.y = node["y"].as<double>();
  if (node["yaw"]) t.yaw = node["yaw"].as<double>();
  return t;
}

std::map<std::string, NavigationTarget> loadNavigationTargets() {
  // 对应 Python: data = _load_yaml('navigation_targets.yaml'); return data.get('targets', {})
  std::map<std::string, NavigationTarget> result;
  YAML::Node data = loadYaml("navigation_targets.yaml");
  if (!data["targets"]) {
    return result;
  }
  YAML::Node targets = data["targets"];
  for (const auto& kv : targets) {
    std::string name = kv.first.as<std::string>();
    result[name] = parseTarget(kv.second);
  }
  return result;
}

bool getNamedTarget(const std::string& name, NavigationTarget& out) {
  // 对应 Python: return load_navigation_targets().get(name, default)
  auto targets = loadNavigationTargets();
  auto it = targets.find(name);
  if (it == targets.end()) {
    return false;
  }
  out = it->second;
  return true;
}

bool getDockTarget(NavigationTarget& out) {
  // 对应 Python: return get_named_target('dock', default)
  return getNamedTarget("dock", out);
}

std::map<std::string, std::vector<NavigationTarget>> loadPatrolRoutes() {
  // 对应 Python: data = _load_yaml('patrol_routes.yaml'); return data.get('routes', {})
  std::map<std::string, std::vector<NavigationTarget>> result;
  YAML::Node data = loadYaml("patrol_routes.yaml");
  if (!data["routes"]) {
    return result;
  }
  YAML::Node routes = data["routes"];
  for (const auto& route_kv : routes) {
    std::string name = route_kv.first.as<std::string>();
    std::vector<NavigationTarget> waypoints;
    const YAML::Node& list = route_kv.second;
    for (const auto& wp : list) {
      waypoints.push_back(parseTarget(wp));
    }
    result[name] = std::move(waypoints);
  }
  return result;
}

std::vector<NavigationTarget> getPatrolRoute(
    const std::string& name,
    const std::vector<NavigationTarget>& default_route) {
  // 对应 Python: route = load_patrol_routes().get(name); if route is None: return list(default or [])
  auto routes = loadPatrolRoutes();
  auto it = routes.find(name);
  if (it == routes.end()) {
    return default_route;
  }
  return it->second;
}

}  // namespace puppy_core
