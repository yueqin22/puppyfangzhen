#!/bin/bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
source ~/puppy_ws/install/setup.bash

echo "=== Robot position (AMCL) ==="
timeout 3 ros2 topic echo /amcl_pose --once 2>&1 | head -15
echo ""
echo "=== Robot position (odom) ==="
timeout 3 ros2 topic echo /odom --once 2>&1 | head -12
echo ""
echo "=== /scan rate ==="
timeout 3 ros2 topic hz /scan 2>&1 | head -3
echo ""
echo "=== /tf rate ==="
timeout 3 ros2 topic hz /tf 2>&1 | head -3
