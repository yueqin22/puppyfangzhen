#!/usr/bin/env python3
"""
将 ROS2 接口脚本附加到 base_footprint 并启动仿真
"""
import sys
import time
sys.path.insert(0, '/home/veni/CoppeliaSim/programming/zmqRemoteApi/clients/python')
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.getObject('sim')

# 获取 base_footprint 句柄
base_footprint = sim.getObject('/base_footprint')
print(f"base_footprint handle: {base_footprint}")

# 创建子脚本
print("创建子脚本...")
script_handle = sim.addScript(sim.scripttype_childscript)
print(f"脚本句柄: {script_handle}")

# 读取 Lua 脚本内容
with open('/mnt/e/puppyfangzhen/puppy_ros2_interface.lua', 'r') as f:
    lua_code = f.read()

# 设置脚本内容
sim.setScriptText(script_handle, lua_code)

# 将脚本关联到 base_footprint
sim.associateScriptWithObject(script_handle, base_footprint)
print("脚本已关联到 base_footprint")

# 保存场景
scene_path = '/home/veni/puppy_ws/src/puppy_worlds/worlds/home_coppelia.ttt'
sim.saveScene(scene_path)
print(f"场景已保存到: {scene_path}")

# 启动仿真
print("启动仿真...")
sim.startSimulation()
time.sleep(2)

# 检查仿真状态
running = sim.getSimulationState()
print(f"仿真状态: {running} (应该为 {sim.simulation_advancing_running})")

print("\n=== 仿真已启动! ===")
print("ROS2 话题应该正在发布:")
print("  /cmd_vel (订阅)")
print("  /scan (发布)")
print("  /odom (发布)")
print("  /tf (发布)")
