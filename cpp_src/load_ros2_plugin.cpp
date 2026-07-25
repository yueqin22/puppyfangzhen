// 通过 ZMQ Remote API 连接 CoppeliaSim 并加载 simROS2 插件
//
// 工具脚本实现，对应 Python 版 load_ros2_plugin.py
#include "load_ros2_plugin.h"

#include <algorithm>
#include <cctype>
#include <iostream>

namespace puppy_bringup {

std::vector<std::string> listSimMethods(const coppeliaSimZMQ& sim,
                                        const std::string& keyword) {
  // 列出可用的方法（等价 Python: dir(sim) 过滤）
  // C++ ZMQ 客户端通过 getApiFuncList 获取方法列表
  std::vector<std::string> methods;
  try {
    std::vector<std::string> all_funcs = sim.getApiFuncList();
    for (const auto& m : all_funcs) {
      // 跳过以 '_' 开头的方法
      if (!m.empty() && m[0] == '_') {
        continue;
      }
      // 关键字（小写）匹配
      std::string lower_m = m;
      std::string lower_kw = keyword;
      std::transform(lower_m.begin(), lower_m.end(), lower_m.begin(),
                     ::tolower);
      std::transform(lower_kw.begin(), lower_kw.end(), lower_kw.begin(),
                     ::tolower);
      if (lower_m.find(lower_kw) != std::string::npos) {
        methods.push_back(m);
      }
    }
  } catch (const std::exception& e) {
    std::cerr << "获取方法列表失败: " << e.what() << std::endl;
  }
  return methods;
}

bool tryCallScriptFunction(coppeliaSimZMQ& sim) {
  // 方法1: callScriptFunction
  // 等价 Python: sim.callScriptFunction('sandbox', -1, 'simROS2 = require("simROS2")', [])
  std::cout << "\n尝试 sim.callScriptFunction..." << std::endl;
  try {
    sim.callScriptFunction("sandbox", -1, "simROS2 = require(\"simROS2\")",
                           std::vector<int>{});
    std::cout << "结果: 成功" << std::endl;
    return true;
  } catch (const std::exception& e) {
    std::cout << "callScriptFunction 失败: " << e.what() << std::endl;
    return false;
  }
}

bool tryExecuteScriptString(coppeliaSimZMQ& sim) {
  // 方法2: 通过 client.call 直接调用 sim.executeScriptString
  // 等价 Python: client.call('sim.executeScriptString', "simROS2 = require('simROS2')", {'scriptHandle': -1})
  std::cout << "\n尝试 client.call..." << std::endl;
  try {
    sim.call("sim.executeScriptString", "simROS2 = require('simROS2')",
             -1 /* scriptHandle */);
    std::cout << "结果: 成功" << std::endl;
    return true;
  } catch (const std::exception& e) {
    std::cout << "client.call 失败: " << e.what() << std::endl;
    return false;
  }
}

int runLoadRos2Plugin() {
  // 连接 CoppeliaSim（coppeliaSimZMQ 构造时建立 ZMQ 连接）
  coppeliaSimZMQ sim;

  // 列出可用的方法（名称包含 'script'）
  std::vector<std::string> script_methods = listSimMethods(sim, "script");
  std::cout << "Script 相关方法:";
  for (const auto& m : script_methods) {
    std::cout << " " << m;
  }
  std::cout << std::endl;

  // 尝试不同的方法执行 Lua 代码
  tryCallScriptFunction(sim);
  tryExecuteScriptString(sim);

  // 检查所有 execute 相关方法
  std::vector<std::string> exec_methods = listSimMethods(sim, "exec");
  std::cout << "\nExecute 相关方法:";
  for (const auto& m : exec_methods) {
    std::cout << " " << m;
  }
  std::cout << std::endl;

  return 0;
}

}  // namespace puppy_bringup

int main() {
  return puppy_bringup::runLoadRos2Plugin();
}
