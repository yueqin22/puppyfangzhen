#!/usr/bin/env python3
"""重启 CoppeliaSim 仿真并验证"""
import sys
import time
sys.path.insert(0, '/home/veni/CoppeliaSim/programming/zmqRemoteApi/clients/python')
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

try:
    client = RemoteAPIClient()
    sim = client.getObject('sim')
    
    state = sim.getSimulationState()
    print(f'当前仿真状态: {state}')
    
    # 停止仿真
    print('停止仿真...')
    sim.stopSimulation()
    time.sleep(3)
    
    # 重启仿真
    print('重启仿真...')
    sim.startSimulation()
    time.sleep(4)
    
    state2 = sim.getSimulationState()
    print(f'重启后状态: {state2}')
    
    if state2 == sim.simulation_advancing_running:
        print('✓ 仿真重启成功！')
        # 检查 base_footprint 位置
        base = sim.getObject('/base_footprint')
        pos = sim.getObjectPosition(base, -1)
        print(f'机器人位置: x={pos[0]:.2f}, y={pos[1]:.2f}, z={pos[2]:.2f}')
    else:
        print('✗ 仿真重启失败')
except Exception as e:
    print(f'错误: {e}')
    import traceback
    traceback.print_exc()
