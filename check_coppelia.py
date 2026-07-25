#!/usr/bin/env python3
"""检查 CoppeliaSim 连接状态和场景对象"""
import sys
sys.path.insert(0, '/home/veni/CoppeliaSim/programming/zmqRemoteApi/clients/python')
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

print('连接 CoppeliaSim...')
client = RemoteAPIClient()
sim = client.getObject('sim')
print('连接成功!')

# 列出场景中的对象
objs = sim.getObjectsInTree(sim.handle_scene)
print(f'场景中对象数量: {len(objs)}')
for obj in objs:
    try:
        alias = sim.getObjectAlias(obj, 3)
        pos = sim.getObjectPosition(obj, -1)
        print(f'  {alias}: handle={obj}, pos={[round(p,2) for p in pos]}')
    except:
        pass
