#!/usr/bin/env python3
"""简化版场景加载和脚本附加 - 带超时"""
import sys
import os
import time

LOG_FILE = '/mnt/e/puppyfangzhen/coppelia_log.txt'

def log(msg):
    line = f"[{time.time():.1f}] {msg}"
    print(line, flush=True)
    with open(LOG_FILE, 'a') as f:
        f.write(line + '\n')

# 清空日志
with open(LOG_FILE, 'w') as f:
    f.write('')

log("=== 开始 ===")

sys.path.insert(0, '/home/veni/CoppeliaSim/programming/zmqRemoteApi/clients/python')
log("导入 ZMQ 模块...")
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

log("创建 client (带超时)...")
client = RemoteAPIClient()
log("获取 sim...")
sim = client.getObject('sim')
log("sim 获取成功!")

log("停止仿真...")
try:
    sim.stopSimulation()
    time.sleep(3)
    log("仿真已停止")
except Exception as e:
    log(f"停止仿真出错（可忽略）: {e}")

log("加载场景...")
scene_path = '/home/veni/puppy_ws/src/puppy_worlds/worlds/home_coppelia.ttt'
sim.loadScene(scene_path)
log("场景已加载")
time.sleep(1)

log("获取 base_footprint...")
base_footprint = sim.getObject('/base_footprint')
log(f"base_footprint handle: {base_footprint}")

log("移除旧脚本...")
try:
    old_script = sim.getScript(sim.scripttype_childscript, base_footprint)
    log(f"旧脚本句柄: {old_script}")
    if old_script >= 0:
        sim.removeScript(old_script)
        time.sleep(0.5)
        log("旧脚本已移除")
    else:
        log("没有旧脚本")
except Exception as e:
    log(f"查询旧脚本出错: {e}")

log("创建新脚本...")
script_handle = sim.addScript(sim.scripttype_childscript)
log(f"脚本句柄: {script_handle}")

log("读取 Lua 代码...")
with open('/mnt/e/puppyfangzhen/puppy_ros2_interface.lua', 'r') as f:
    lua_code = f.read()
log(f"Lua 代码长度: {len(lua_code)}")

log("设置脚本文本...")
sim.setScriptText(script_handle, lua_code)

log("关联脚本到对象...")
sim.associateScriptWithObject(script_handle, base_footprint)

log("保存场景...")
sim.saveScene(scene_path)

log("启动仿真...")
sim.startSimulation()
time.sleep(3)

state = sim.getSimulationState()
log(f"仿真状态: {state} (运行中={sim.simulation_advancing_running})")

if state == sim.simulation_advancing_running:
    log("✓ 仿真运行成功！")
else:
    log("✗ 仿真未运行")

log("=== 完成 ===")
