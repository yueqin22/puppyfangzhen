#!/bin/bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
source ~/puppy_ws/install/setup.bash

echo "=== All topics ==="
ros2 topic list 2>&1
echo ""
echo "=== /scan info ==="
ros2 topic info /scan -v 2>&1 | head -15
echo ""
echo "=== /odom info ==="
ros2 topic info /odom -v 2>&1 | head -10
echo ""
echo "=== TF: odom -> base_footprint ==="
timeout 3 ros2 run tf2_ros tf2_echo odom base_footprint 2>&1 | head -5
echo ""
echo "=== TF: base_link -> laser_link ==="
timeout 3 ros2 run tf2_ros tf2_echo base_link laser_link 2>&1 | head -5
echo ""
echo "=== Node list ==="
ros2 node list 2>&1
echo ""
echo "=== /scan hz (2s) ==="
timeout 2 ros2 topic hz /scan 2>&1 | head -3
