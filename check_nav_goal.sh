#!/bin/bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
source ~/puppy_ws/install/setup.bash

echo "=== Action servers ==="
ros2 action list 2>&1

echo "=== Action server status ==="
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: -2.0, z: 0.0}, orientation: {w: 1.0}}}}" --feedback 2>&1 | head -10

echo "=== Lifecycle states ==="
for node in map_server amcl controller_server planner_server behavior_server bt_navigator; do
  echo -n "$node: "
  ros2 lifecycle get /$node 2>&1
done

echo "=== /goal_pose topic ==="
ros2 topic info /goal_pose -v 2>&1 | head -15
