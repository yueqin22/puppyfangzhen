#!/bin/bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
source ~/puppy_ws/install/setup.bash

echo "=== /clock (sim time) ==="
timeout 2 ros2 topic echo /clock --once 2>&1 | head -5
echo ""
echo "=== /tf content (3s) ==="
timeout 3 ros2 topic echo /tf 2>&1 | head -40
echo ""
echo "=== /tf_static content (3s) ==="
timeout 3 ros2 topic echo /tf_static 2>&1 | head -30
echo ""
echo "=== /odom data ==="
timeout 3 ros2 topic echo /odom --once 2>&1 | head -15
echo ""
echo "=== /scan data (5s wait) ==="
timeout 5 ros2 topic echo /scan --once 2>&1 | head -15
echo ""
echo "=== Gazebo performance ==="
timeout 2 ros2 topic echo /performance_metrics --once 2>&1 | head -20
