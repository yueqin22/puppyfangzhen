#!/bin/bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
source ~/puppy_ws/install/setup.bash

echo "=== Sending goal to (0.0, 1.5) - through doorway ==="
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 0.0, y: 1.5, z: 0.0}, orientation: {z: 0.7071, w: 0.7071}}}}" --feedback 2>&1 | tail -30
