#!/bin/bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
source ~/puppy_ws/install/setup.bash

echo "=== Battery State ==="
timeout 3 ros2 topic echo /battery_state --once 2>&1 | head -15

echo "=== Robot Position ==="
timeout 3 ros2 topic echo /amcl_pose --once 2>&1 | grep -A3 "position:" | head -4

echo "=== Scan rate ==="
timeout 3 ros2 topic hz /scan 2>&1 | head -3

echo "=== Odom rate ==="
timeout 3 ros2 topic hz /odom 2>&1 | head -3

echo "=== Node list ==="
ros2 node list 2>&1 | sort

echo "=== Active goals ==="
ros2 action list 2>&1
