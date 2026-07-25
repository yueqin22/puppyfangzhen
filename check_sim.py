#!/usr/bin/env python3
"""检查 CoppeliaSim 仿真状态"""
import sys
sys.path.insert(0, '/home/veni/CoppeliaSim/programming/zmqRemoteApi/clients/python')
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

try:
    client = RemoteAPIClient()
    sim = client.getObject('sim')
    state = sim.getSimulationState()
    print(f'仿真状态: {state} (运行中={sim.simulation_advancing_running})')
    time = sim.getSimulationTime()
    print(f'仿真时间: {time}')
    # 读取机器人位置
    base = sim.getObject('/base_footprint')
    pos = sim.getObjectPosition(base, -1)
    print(f'机器人位置: x={pos[0]:.2f}, y={pos[1]:.2f}, z={pos[2]:.2f}')
except Exception as e:
    print(f'ZMQ错误: {e}')
