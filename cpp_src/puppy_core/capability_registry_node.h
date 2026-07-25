// Capability Registry Node: ROS2 node wrapper for CapabilityRegistry.
//
// 将系统能力注册表通过 ROS2 topic + service 暴露出来，
// 供其他模块查询与降级使用。
#pragma once

#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "puppy_interfaces/msg/robot_health.hpp"

namespace puppy_core {

/// 单个系统能力描述
struct Capability {
  std::string name;
  bool enabled{true};
  bool available{true};
  std::string reason;
};

/// 能力注册表：声明并查询系统能力，支持优雅降级
class CapabilityRegistry {
 public:
  CapabilityRegistry();

  void registerCapability(const std::string& name,
                          bool enabled = true,
                          bool available = true);

  bool isAvailable(const std::string& name) const;

  void setEnabled(const std::string& name, bool enabled,
                  const std::string& reason = "");

  void disable(const std::string& name, const std::string& reason = "");
  void enable(const std::string& name);

  /// 返回 {name: {enabled, available, reason}} 形式的状态
  std::map<std::string, std::map<std::string, std::string>> getStatus() const;
  /// 返回当前 enabled && available 的能力名集合
  std::set<std::string> getAvailable() const;

 private:
  std::map<std::string, Capability> caps_;
};

/// ROS2 node that exposes CapabilityRegistry via topic + service.
class CapabilityRegistryNode : public rclcpp::Node {
 public:
  CapabilityRegistryNode();
  ~CapabilityRegistryNode() override = default;

 private:
  void onHealth(const puppy_interfaces::msg::RobotHealth::SharedPtr msg);
  void publishCapabilities();
  void onQuery(
      const std_srvs::srv::Trigger::Request::SharedPtr request,
      std_srvs::srv::Trigger::Response::SharedPtr response);

  static const std::vector<std::string> CAP_NAMES;

  CapabilityRegistry registry_;

  rclcpp::Subscription<puppy_interfaces::msg::RobotHealth>::SharedPtr
      health_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cap_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr query_srv_;
};

}  // namespace puppy_core
