#!/bin/bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
source ~/puppy_ws/install/setup.bash

echo "=== Robot position (AMCL) ==="
timeout 3 ros2 topic echo /amcl_pose --once 2>&1 | grep -A5 "position:" | head -6

echo "=== Robot position (TF) ==="
timeout 3 ros2 run tf2_ros tf2_echo map base_footprint 2>&1 | head -5

echo "=== Global costmap metadata ==="
timeout 3 ros2 topic echo /global_costmap/costmap_updates --once 2>&1 | head -10

echo "=== Check costmap at robot pos and targets ==="
python3 -c "
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
from nav2_msgs.msg import Costmap
import sys

rclpy.init()
node = Node('checker')
costmap = None
def cb(msg):
    global costmap
    costmap = msg
qos = QoSProfile(depth=1, history=QoSHistoryPolicy.KEEP_LAST, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
sub = node.create_subscription(Costmap, '/global_costmap/costmap_raw', cb, qos)
import time
start = time.time()
while costmap is None and time.time() - start < 5:
    rclpy.spin_once(node, timeout_sec=0.1)
if costmap is None:
    print('No costmap received!')
    sys.exit(1)
m = costmap
print(f'Costmap: {m.metadata.size_x}x{m.metadata.size_y}, res={m.metadata.resolution}, origin=({m.metadata.origin.position.x},{m.metadata.origin.position.y})')
print(f'Update time: {m.metadata.update_time.sec}.{m.metadata.update_time.nanosec}')
ox = m.metadata.origin.position.x
oy = m.metadata.origin.position.y
res = m.metadata.resolution
sx = m.metadata.size_x
sy = m.metadata.size_y
def check(x, y, name):
    mx = int((x - ox) / res)
    my = int((y - oy) / res)
    if 0 <= mx < sx and 0 <= my < sy:
        idx = my * sx + mx
        c = m.data[idx]
        print(f'  {name}({x},{y}): cost={c} [{\"FREE\" if c==0 else \"OCCUPIED\" if c>=254 else \"INFLATED\"}]')
    else:
        print(f'  {name}({x},{y}): OUT OF BOUNDS (mx={mx},my={my},size={sx}x{sy})')
check(0, 1.5, 'waypoint3/5')
check(-2.5, 1.5, 'waypoint4')
check(0, 0, 'doorway')
check(2, -2, 'living')
check(3, 2, 'kitchen')
node.destroy_node()
rclpy.shutdown()
" 2>&1
