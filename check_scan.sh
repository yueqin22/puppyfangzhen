#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/veni/puppy_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

echo "=== /scan topic info ==="
ros2 topic info /scan 2>&1

echo ""
echo "=== /odom topic (1 msg) ==="
timeout 3 ros2 topic echo /odom --once 2>&1 | head -20

echo ""
echo "=== /cmd_vel topic info ==="
ros2 topic info /cmd_vel 2>&1

echo ""
echo "=== All topics ==="
ros2 topic list 2>&1 | sort

echo ""
echo "=== Gazebo model list ==="
timeout 5 ros2 service call /get_model_state gazebo_msgs/srv/GetModelState "{model_name: 'puppy_robot'}" 2>&1 | head -20
