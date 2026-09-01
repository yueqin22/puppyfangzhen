#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/veni/puppy_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

echo "=== Current Robot State ==="
echo "--- /odom ---"
timeout 3 ros2 topic echo /odom --once 2>&1 | grep -A 8 "position:" | head -10

echo ""
echo "--- TF: map -> base_footprint ---"
timeout 5 ros2 run tf2_ros tf2_echo map base_footprint 2>&1 | tail -8

echo ""
echo "=== Sending Test Goal (1.0, 0.0, 0.0) ==="
python3 /mnt/e/puppyfangzhen/send_goal.py 1.0 0.0 0.0 2>&1

echo ""
echo "=== Nav2 Log (last 30 lines) ==="
tail -30 /home/veni/nav2.log 2>/dev/null

echo ""
echo "=== /cmd_vel (check if velocity is being sent) ==="
timeout 3 ros2 topic echo /cmd_vel --once 2>&1 | head -10
