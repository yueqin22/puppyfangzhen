#!/bin/bash
export DISPLAY=:1
export COPPELIASIM_ROOT_DIR=/home/veni/CoppeliaSim
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash

python3 -u << 'PYEOF'
import sys
import traceback
sys.path.insert(0, '/home/veni/CoppeliaSim/programming/zmqRemoteApi/clients/python')

print("1. 导入 zmqRemoteApi...")
try:
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient
    print("   成功!")
except Exception as e:
    print(f"   失败: {e}")
    traceback.print_exc()
    sys.exit(1)

print("2. 连接 CoppeliaSim...")
try:
    client = RemoteAPIClient()
    print("   连接建立!")
except Exception as e:
    print(f"   失败: {e}")
    traceback.print_exc()
    sys.exit(1)

print("3. 获取 sim 对象...")
try:
    sim = client.getObject('sim')
    print("   成功!")
except Exception as e:
    print(f"   失败: {e}")
    traceback.print_exc()
    sys.exit(1)

print("4. 列出场景对象...")
try:
    objs = sim.getObjectsInTree(sim.handle_scene)
    print(f"   场景中对象数量: {len(objs)}")
    for obj in objs[:10]:
        try:
            alias = sim.getObjectAlias(obj, 3)
            print(f"     {alias}: handle={obj}")
        except:
            pass
except Exception as e:
    print(f"   失败: {e}")
    traceback.print_exc()

print("完成!")
PYEOF
echo "SCRIPT_EXIT: $?"
