#!/usr/bin/env python3
"""检查 sim 可用的常量"""
import sys
sys.path.insert(0, '/home/veni/CoppeliaSim/programming/zmqRemoteApi/clients/python')
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.getObject('sim')

# 检查 primitive shape 相关常量
prim_attrs = [m for m in dir(sim) if 'primitive' in m.lower() or 'shape' in m.lower()]
print("Shape 相关:", prim_attrs[:20])

# 检查 createPrimitiveShape 的签名
print("\ncreatePrimitiveShape:", sim.createPrimitiveShape.__doc__ if hasattr(sim.createPrimitiveShape, '__doc__') else 'no doc')

# 尝试不同的常量名
for name in ['primitiveshape_rectangle', 'primitiveshape_cuboid', 'primitiveshape_box',
             'shape_rectangle', 'shape_box', 'shape_cuboid']:
    val = getattr(sim, name, None)
    print(f"sim.{name} = {val}")

# 直接用数字尝试
print("\n尝试用数字常量...")
for val in [0, 1, 2, 3, 4]:
    try:
        shape = sim.createPrimitiveShape(val, [0.1, 0.1, 0.1])
        print(f"  值 {val}: 成功! handle={shape}")
        sim.removeObject(shape)
    except Exception as e:
        print(f"  值 {val}: 失败 - {e}")
