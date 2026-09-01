#!/bin/bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
source ~/puppy_ws/install/setup.bash

echo "=== /scan topic info ==="
ros2 topic info /scan -v 2>&1 | head -15
echo ""
echo "=== /scan data (first message) ==="
timeout 3 ros2 topic echo /scan --once --field header 2>&1 | head -10
echo ""
echo "=== /odom topic info ==="
ros2 topic info /odom -v 2>&1 | head -15
echo ""
echo "=== TF frames ==="
timeout 3 ros2 run tf2_tools view_frames 2>&1 | head -5
echo ""
echo "=== TF: odom -> base_footprint ==="
timeout 3 ros2 run tf2_ros tf2_echo odom base_footprint 2>&1 | head -10
echo ""
echo "=== AMCL node info ==="
ros2 node info /amcl 2>&1 | head -30
