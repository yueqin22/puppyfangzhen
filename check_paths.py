#!/usr/bin/env python3
import sys
sys.path.insert(0, '/home/veni/CoppeliaSim/programming/zmqRemoteApi/clients/python')
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.getObject('sim')

# 列出所有对象及其路径
objs = sim.getObjectsInTree(sim.handle_scene)
print(f"对象数量: {len(objs)}")
for obj in objs:
    try:
        alias = sim.getObjectAlias(obj, 3)
        # 尝试不同的路径格式
        path1 = sim.getObjectAlias(obj, 1)  # 短名
        path2 = sim.getObjectAlias(obj, 2)  # 完整路径
        path3 = sim.getObjectAlias(obj, 0)  # 默认
        print(f"  handle={obj}: alias3={alias}, alias1={path1}, alias2={path2}, alias0={path3}")
    except Exception as e:
        print(f"  handle={obj}: 错误 - {e}")

# 尝试用不同格式获取 base_footprint
print("\n尝试获取 base_footprint:")
for path in ['base_footprint', '/base_footprint', './base_footprint', '::base_footprint', '//base_footprint']:
    try:
        h = sim.getObject(path)
        print(f"  '{path}': 成功! handle={h}")
    except Exception as e:
        print(f"  '{path}': 失败 - {e}")
