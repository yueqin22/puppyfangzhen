// Capability Registry 实现
//
// 纯工具类（非 ROS2 节点），对应 Python: puppy_core/capability_registry.py
#include "puppy_core/capability_registry.h"

namespace puppy_core {
namespace registry {

CapabilityRegistry::CapabilityRegistry() {
  registerCapability("navigation");
  registerCapability("patrol");
  registerCapability("security");
  registerCapability("emotion");
  registerCapability("fall_detection");
  registerCapability("docking");
  registerCapability("rgb_camera");
  registerCapability("depth_camera");
  registerCapability("lidar");
  registerCapability("imu");
}

void CapabilityRegistry::registerCapability(const std::string& name,
                                            bool enabled, bool available) {
  Capability cap;
  cap.name = name;
  cap.enabled = enabled;
  cap.available = available;
  caps_[name] = cap;
}

bool CapabilityRegistry::isAvailable(const std::string& name) const {
  auto it = caps_.find(name);
  if (it == caps_.end()) {
    return false;
  }
  return it->second.enabled && it->second.available;
}

void CapabilityRegistry::setEnabled(const std::string& name, bool enabled,
                                    const std::string& reason) {
  auto it = caps_.find(name);
  if (it == caps_.end()) {
    return;
  }
  it->second.enabled = enabled;
  if (!enabled) {
    it->second.reason = reason.empty() ? "disabled by config" : reason;
  } else if (it->second.available) {
    it->second.reason = "";
  }
}

void CapabilityRegistry::disable(const std::string& name,
                                 const std::string& reason) {
  auto it = caps_.find(name);
  if (it == caps_.end()) {
    return;
  }
  it->second.available = false;
  it->second.reason = reason;
}

void CapabilityRegistry::enable(const std::string& name) {
  auto it = caps_.find(name);
  if (it == caps_.end()) {
    return;
  }
  if (it->second.enabled) {
    it->second.available = true;
    it->second.reason = "";
  }
}

std::map<std::string, std::map<std::string, std::string>>
CapabilityRegistry::getStatus() const {
  std::map<std::string, std::map<std::string, std::string>> out;
  for (const auto& kv : caps_) {
    const Capability& c = kv.second;
    auto& entry = out[kv.first];
    entry["enabled"] = c.enabled ? "true" : "false";
    entry["available"] = c.available ? "true" : "false";
    entry["reason"] = c.reason;
  }
  return out;
}

std::set<std::string> CapabilityRegistry::getAvailable() const {
  std::set<std::string> out;
  for (const auto& kv : caps_) {
    if (kv.second.enabled && kv.second.available) {
      out.insert(kv.first);
    }
  }
  return out;
}

}  // namespace registry
}  // namespace puppy_core
