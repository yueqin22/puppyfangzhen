#!/usr/bin/env python3
"""检查并重启 CoppeliaSim 仿真"""
import sys
import time
sys.path.insert(0, '/home/veni/CoppeliaSim/programming/zmqRemoteApi/clients/python')
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

try:
    client = RemoteAPIClient()
    sim = client.getObject('sim')
    state = sim.getSimulationState()
    print(f'仿真状态: {state}')
    print(f'运行中(17)={sim.simulation_advancing_running}')
    if state != sim.simulation_advancing_running:
        print('仿真未运行，正在重启...')
        sim.startSimulation()
        time.sleep(3)
        state2 = sim.getSimulationState()
        print(f'重启后状态: {state2}')
        if state2 == sim.simulation_advancing_running:
            print('✓ 仿真重启成功！')
        else:
            print('✗ 仿真重启失败')
    else:
        print('仿真正常运行中')
except Exception as e:
    print(f'错误: {e}')
