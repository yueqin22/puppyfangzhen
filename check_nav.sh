#!/bin/bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
source ~/puppy_ws/install/setup.bash

echo "=== AMCL Pose ==="
timeout 3 ros2 topic echo /amcl_pose --once 2>&1 | head -20
echo ""
echo "=== TF: map -> base_footprint ==="
timeout 3 ros2 run tf2_ros tf2_echo map base_footprint 2>&1 | head -8
echo ""
echo "=== /scan hz ==="
timeout 3 ros2 topic hz /scan 2>&1 | head -3
echo ""
echo "=== Global costmap at robot pos ==="
timeout 3 ros2 topic echo /global_costmap/costmap_raw --once 2>&1 | head -5
