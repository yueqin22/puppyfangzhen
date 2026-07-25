#!/usr/bin/env python3
"""停止仿真、重新附加修复后的Lua脚本、重启仿真"""
import sys
import time
sys.path.insert(0, '/home/veni/CoppeliaSim/programming/zmqRemoteApi/clients/python')
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.getObject('sim')

# 0. 加载已保存的场景
scene_path = '/home/veni/puppy_ws/src/puppy_worlds/worlds/home_coppelia.ttt'
import os
if os.path.exists(scene_path):
    print(f"加载场景: {scene_path}")
    sim.loadScene(scene_path)
    time.sleep(2)
    print("场景已加载")
else:
    print(f"场景文件不存在: {scene_path}")
    print("请先运行 build_coppelia_scene.py 构建场景")
    sys.exit(1)

# 1. 停止当前仿真
print("停止当前仿真...")
try:
    sim.stopSimulation()
    time.sleep(2)
    print("仿真已停止")
except Exception as e:
    print(f"停止仿真时出错（可忽略）: {e}")

# 2. 获取 base_footprint
base_footprint = sim.getObject('/base_footprint')
print(f"base_footprint handle: {base_footprint}")

# 3. 移除旧的子脚本（如果存在）
try:
    old_script = sim.getScript(sim.scripttype_childscript, base_footprint)
    if old_script >= 0:
        print(f"移除旧脚本: {old_script}")
        sim.removeScript(old_script)
        time.sleep(0.5)
    else:
        print("没有旧脚本需要移除")
except Exception as e:
    print(f"查询旧脚本时出错（可忽略）: {e}")

# 4. 创建新脚本并附加
script_handle = sim.addScript(sim.scripttype_childscript)
print(f"新脚本句柄: {script_handle}")

with open('/mnt/e/puppyfangzhen/puppy_ros2_interface.lua', 'r') as f:
    lua_code = f.read()

sim.setScriptText(script_handle, lua_code)
sim.associateScriptWithObject(script_handle, base_footprint)
print("脚本已关联到 base_footprint")

# 5. 保存场景
scene_path = '/home/veni/puppy_ws/src/puppy_worlds/worlds/home_coppelia.ttt'
sim.saveScene(scene_path)
print(f"场景已保存: {scene_path}")

# 6. 启动仿真
print("启动仿真...")
sim.startSimulation()
time.sleep(3)

# 7. 检查仿真状态
state = sim.getSimulationState()
print(f"仿真状态: {state} (运行中={sim.simulation_advancing_running})")

if state == sim.simulation_advancing_running:
    print("✓ 仿真运行成功！")
else:
    print("✗ 仿真未运行，请检查日志")

print("完成")
