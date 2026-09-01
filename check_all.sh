#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/veni/puppy_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

echo "=== /scan topic info ==="
ros2 topic info /scan -v 2>&1

echo ""
echo "=== /scan (wait 5s) ==="
timeout 5 ros2 topic echo /scan --once 2>&1 | head -20

echo ""
echo "=== /odom (1 msg) ==="
timeout 5 ros2 topic echo /odom --once 2>&1 | head -15

echo ""
echo "=== /clock ==="
timeout 3 ros2 topic echo /clock --once 2>&1 | head -5

echo ""
echo "=== Gazebo log (last 20 lines) ==="
tail -20 /home/veni/gazebo.log 2>/dev/null

echo ""
echo "=== Nav2 log (last 20 lines) ==="
tail -20 /home/veni/nav2.log 2>/dev/null

echo ""
echo "=== Process list ==="
ps aux | grep -E 'gzserver|rviz|x11vnc|Xvfb|nav2' | grep -v grep | head -15
