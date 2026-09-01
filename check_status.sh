#!/bin/bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
source ~/puppy_ws/install/setup.bash

echo "=== Topic List ==="
ros2 topic list 2>&1 | head -40
echo ""
echo "=== AMCL Pose ==="
timeout 3 ros2 topic echo /amcl_pose --once 2>&1 | head -15
echo ""
echo "=== Odometry ==="
timeout 3 ros2 topic echo /odom --once 2>&1 | head -15
echo ""
echo "=== TF: map -> base_footprint ==="
timeout 3 ros2 run tf2_ros tf2_echo map base_footprint 2>&1 | head -10
