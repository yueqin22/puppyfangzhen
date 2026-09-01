#!/bin/bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
source ~/puppy_ws/install/setup.bash
for node in map_server amcl controller_server planner_server behavior_server bt_navigator; do
  echo -n "$node: "
  ros2 lifecycle get /$node 2>&1
done
echo "---"
echo "Node list:"
ros2 node list 2>&1 | head -20
