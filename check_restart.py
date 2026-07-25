#!/usr/bin/env python3
"""检查并重启仿真"""
import sys
import time
sys.path.insert(0, '/home/veni/CoppeliaSim/programming/zmqRemoteApi/clients/python')
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.getObject('sim')

state = sim.getSimulationState()
print(f'仿真状态: {state} (运行中={sim.simulation_advancing_running})')

if state != sim.simulation_advancing_running:
    print('仿真未运行，正在重启...')
    sim.startSimulation()
    time.sleep(3)
    state = sim.getSimulationState()
    print(f'重启后状态: {state} (运行中={sim.simulation_advancing_running})')
else:
    print('仿真正在运行')

# 检查脚本是否还在
base_footprint = sim.getObject('/base_footprint')
script_handle = sim.getScript(sim.scripttype_childscript, base_footprint)
print(f'脚本句柄: {script_handle}')

sim_time = sim.getSimulationTime()
print(f'仿真时间: {sim_time}')
