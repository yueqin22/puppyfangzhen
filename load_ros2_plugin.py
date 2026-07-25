#!/usr/bin/env python3
"""通过 ZMQ Remote API 连接 CoppeliaSim 并加载 simROS2 插件"""
import sys
sys.path.insert(0, '/home/veni/CoppeliaSim/programming/zmqRemoteApi/clients/python')
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.getObject('sim')

# 列出可用的方法
methods = [m for m in dir(sim) if not m.startswith('_') and 'script' in m.lower()]
print("Script 相关方法:", methods)

# 尝试不同的方法执行 Lua 代码
try:
    # 方法1: callScriptFunction
    print("\n尝试 sim.callScriptFunction...")
    result = sim.callScriptFunction('sandbox', -1, 'simROS2 = require("simROS2")', [])
    print(f"结果: {result}")
except Exception as e:
    print(f"callScriptFunction 失败: {e}")

try:
    # 方法2: 通过 client.call 直接调用
    print("\n尝试 client.call...")
    result = client.call('sim.executeScriptString', "simROS2 = require('simROS2')", {'scriptHandle': -1})
    print(f"结果: {result}")
except Exception as e:
    print(f"client.call 失败: {e}")

# 检查所有 execute 相关方法
exec_methods = [m for m in dir(sim) if not m.startswith('_') and 'exec' in m.lower()]
print("\nExecute 相关方法:", exec_methods)
