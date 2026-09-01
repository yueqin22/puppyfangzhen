#!/bin/bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
source ~/puppy_ws/install/setup.bash

echo "=== Robot AMCL pose ==="
timeout 5 ros2 topic echo /amcl_pose --once 2>&1 | grep -A5 "position:"
echo ""
echo "=== Robot odom ==="
timeout 5 ros2 topic echo /odom --once 2>&1 | grep -A5 "position:"
echo ""
echo "=== Global costmap metadata ==="
timeout 5 ros2 topic echo /global_costmap/costmap_raw --once 2>&1 | grep -A10 "metadata:"
