#!/bin/bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
source ~/puppy_ws/install/setup.bash

echo "=== AMCL lifecycle ==="
ros2 lifecycle get /amcl 2>&1

echo "=== AMCL node info ==="
ros2 node info /amcl 2>&1 | head -30

echo "=== TF frames ==="
ros2 run tf2_tools view_frames 2>&1 | head -5

echo "=== /tf topics ==="
ros2 topic list 2>&1 | grep tf

echo "=== AMCL params (key ones) ==="
ros2 param get /amcl tf_broadcast 2>&1
ros2 param get /amcl save_pose_rate 2>&1
ros2 param get /amcl transform_tolerance 2>&1
ros2 param get /amcl global_frame_id 2>&1
ros2 param get /amcl odom_frame_id 2>&1

echo "=== Check /tf publishers ==="
ros2 topic info /tf -v 2>&1 | head -30

echo "=== AMCL subscription to /scan ==="
timeout 3 ros2 topic hz /scan 2>&1 | head -3
