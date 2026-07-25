// Capability Registry Node 实现
#include "puppy_core/capability_registry_node.h"

#include <sstream>

namespace puppy_core {

// 能力名称常量（与Python CAP_NAMES 一致）
const std::vector<std::string> CapabilityRegistryNode::CAP_NAMES = {
    "navigation", "patrol",   "security",       "emotion",
    "fall_detection", "docking", "rgb_camera", "depth_camera",
    "lidar",      "imu",
};

// ---------------------------------------------------------------------------
// CapabilityRegistry
// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
// 简易 JSON 序列化（避免引入 nlohmann/json 依赖）
// 输出格式： {"name":{"enabled":true,"available":false,"reason":"..."}, ...}
// ---------------------------------------------------------------------------
static std::string statusToJson(
    const std::map<std::string, std::map<std::string, std::string>>& status) {
  std::ostringstream oss;
  oss << "{";
  bool first = true;
  for (const auto& kv : status) {
    if (!first) {
      oss << ", ";
    }
    first = false;
    oss << "\"" << kv.first << "\": {";
    oss << "\"enabled\": " << kv.second.at("enabled") << ", ";
    oss << "\"available\": " << kv.second.at("available") << ", ";
    oss << "\"reason\": \"" << kv.second.at("reason") << "\"";
    oss << "}";
  }
  oss << "}";
  return oss.str();
}

// ---------------------------------------------------------------------------
// CapabilityRegistryNode
// ---------------------------------------------------------------------------
CapabilityRegistryNode::CapabilityRegistryNode()
    : rclcpp::Node("capability_registry") {
  for (const auto& cap : CAP_NAMES) {
    this->declare_parameter("enable_" + cap, true);
    if (!this->get_parameter("enable_" + cap).as_bool()) {
      registry_.setEnabled(cap, false, "disabled by config");
    }
  }

  health_sub_ =
      this->create_subscription<puppy_interfaces::msg::RobotHealth>(
          "/robot/health", 10,
          std::bind(&CapabilityRegistryNode::onHealth, this,
                    std::placeholders::_1));

  cap_pub_ = this->create_publisher<std_msgs::msg::String>(
      "/robot/capabilities", 10);
  timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&CapabilityRegistryNode::publishCapabilities, this));

  query_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "/robot/capability_query",
      std::bind(&CapabilityRegistryNode::onQuery, this,
                std::placeholders::_1, std::placeholders::_2));

  // 列出当前可用能力，与Python版日志保持一致
  std::set<std::string> available = registry_.getAvailable();
  std::ostringstream oss;
  bool first = true;
  for (const auto& name : available) {
    if (!first) {
      oss << ", ";
    }
    first = false;
    oss << name;
  }
  RCLCPP_INFO(this->get_logger(),
              "Capability registry started (available: [%s])",
              oss.str().c_str());
}

void CapabilityRegistryNode::onHealth(
    const puppy_interfaces::msg::RobotHealth::SharedPtr msg) {
  if (!msg->lidar_ready) {
    registry_.disable("lidar", "LIDAR offline or timeout");
    registry_.disable("patrol", "patrol requires LIDAR");
  } else {
    registry_.enable("lidar");
    if (registry_.isAvailable("navigation")) {
      registry_.enable("patrol");
    }
  }

  if (!msg->imu_ready) {
    registry_.disable("imu", "IMU offline or timeout");
    registry_.disable("navigation", "navigation requires IMU");
  } else {
    registry_.enable("imu");
    registry_.enable("navigation");
  }

  if (!msg->camera_ready) {
    registry_.disable("rgb_camera", "camera offline");
    registry_.disable("depth_camera", "depth camera offline");
  } else {
    registry_.enable("rgb_camera");
    registry_.enable("depth_camera");
  }

  if (!msg->motion_ready) {
    registry_.disable("docking", "motion not controllable");
  } else {
    registry_.enable("docking");
  }
}

void CapabilityRegistryNode::publishCapabilities() {
  std_msgs::msg::String msg;
  msg.data = statusToJson(registry_.getStatus());
  cap_pub_->publish(msg);
}

void CapabilityRegistryNode::onQuery(
    const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
    std_srvs::srv::Trigger::Response::SharedPtr response) {
  response->success = true;
  response->message = statusToJson(registry_.getStatus());
}

}  // namespace puppy_core
