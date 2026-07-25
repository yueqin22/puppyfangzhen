#!/usr/bin/env python3
"""检查仿真状态和ROS2节点"""
import sys
import time
sys.path.insert(0, '/home/veni/CoppeliaSim/programming/zmqRemoteApi/clients/python')
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.getObject('sim')

state = sim.getSimulationState()
print(f"仿真状态: {state} (运行中={sim.simulation_advancing_running})")

# 检查 base_footprint 脚本
base_footprint = sim.getObject('/base_footprint')
script_handle = sim.getScript(sim.scripttype_childscript, base_footprint)
print(f"脚本句柄: {script_handle}")

# 获取仿真时间
sim_time = sim.getSimulationTime()
print(f"仿真时间: {sim_time}")

print("检查完成")
