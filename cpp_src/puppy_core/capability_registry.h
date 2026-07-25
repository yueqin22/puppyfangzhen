// Capability Registry: 声明并查询系统能力 (C++ 版本)
//
// 纯工具类（非 ROS2 节点），对应 Python: puppy_core/capability_registry.py
//
// 注意：为避免与已有的 capability_registry_node.h 中的 CapabilityRegistry 类
// 产生符号冲突（两者都会被编译进同一个 puppy_core_cpp 库），本转换放在
// puppy_core::registry 子命名空间下。
#pragma once

#include <map>
#include <set>
#include <string>

namespace puppy_core {
namespace registry {

/// 单个系统能力描述（对应 Python @dataclass Capability）
struct Capability {
  std::string name;
  bool enabled{true};
  bool available{true};
  std::string reason;
};

/// 能力注册表：声明并查询系统能力，支持优雅降级
/// 对应 Python: CapabilityRegistry
class CapabilityRegistry {
 public:
  CapabilityRegistry();

  // 注册一个能力
  void registerCapability(const std::string& name,
                          bool enabled = true,
                          bool available = true);

  // 查询能力是否可用（存在 && enabled && available）
  bool isAvailable(const std::string& name) const;

  // 设置能力的 enabled 状态
  void setEnabled(const std::string& name, bool enabled,
                  const std::string& reason = "");

  // 禁用能力（标记为不可用）
  void disable(const std::string& name, const std::string& reason = "");

  // 启用能力（标记为可用）
  void enable(const std::string& name);

  /// 返回 {name: {enabled, available, reason}} 形式的状态
  std::map<std::string, std::map<std::string, std::string>> getStatus() const;

  /// 返回当前 enabled && available 的能力名集合
  std::set<std::string> getAvailable() const;

 private:
  std::map<std::string, Capability> caps_;
};

}  // namespace registry
}  // namespace puppy_core
