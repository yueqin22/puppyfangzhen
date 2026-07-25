// 通过 ZMQ Remote API 连接 CoppeliaSim 并加载 simROS2 插件
//
// 工具脚本（非 ROS2 节点），探索 CoppeliaSim ZMQ API 中可用的脚本执行方法。
// 依赖：CoppeliaSim ZMQ Remote API C++ 客户端 (coppeliaSimZMQ.hpp)
//       需要将 CoppeliaSim/programming/zmqRemoteApi/clients/cpp 加入 include 路径
#pragma once

#include <string>
#include <vector>

#include "coppeliaSimZMQ.hpp"

namespace puppy_bringup {

/// 列出 sim 对象上名称包含关键字的可用方法
/// 等价 Python: [m for m in dir(sim) if not m.startswith('_') and keyword in m.lower()]
std::vector<std::string> listSimMethods(const coppeliaSimZMQ& sim,
                                        const std::string& keyword);

/// 尝试通过 callScriptFunction 加载 simROS2 插件
/// 等价 Python: sim.callScriptFunction('sandbox', -1, 'simROS2 = require("simROS2")', [])
bool tryCallScriptFunction(coppeliaSimZMQ& sim);

/// 尝试通过 client.call('sim.executeScriptString', ...) 加载 simROS2 插件
/// 等价 Python: client.call('sim.executeScriptString', "simROS2 = require('simROS2')", {'scriptHandle': -1})
bool tryExecuteScriptString(coppeliaSimZMQ& sim);

/// 运行完整的 simROS2 插件加载探索流程
int runLoadRos2Plugin();

}  // namespace puppy_bringup
