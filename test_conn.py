#!/usr/bin/env python3
import sys
import traceback
sys.path.insert(0, '/home/veni/CoppeliaSim/programming/zmqRemoteApi/clients/python')

print("1. 导入 zmqRemoteApi...")
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
print("   成功!")

print("2. 连接 CoppeliaSim...")
client = RemoteAPIClient()
sim = client.getObject('sim')
print("   连接成功!")

print("3. 列出场景对象...")
objs = sim.getObjectsInTree(sim.handle_scene)
print(f"   场景中对象数量: {len(objs)}")
for obj in objs[:20]:
    try:
        alias = sim.getObjectAlias(obj, 3)
        print(f"     {alias}: handle={obj}")
    except:
        pass

print("4. 测试创建形状...")
try:
    test_shape = sim.createPrimitiveShape(3, [0.1, 0.1, 0.1])
    print(f"   创建成功! handle={test_shape}")
    sim.removeObject(test_shape)
    print("   删除成功!")
except Exception as e:
    print(f"   失败: {e}")
    traceback.print_exc()

print("完成!")
