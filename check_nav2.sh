#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/veni/puppy_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

echo "=== Lifecycle States ==="
for node in map_server amcl planner_server controller_server bt_navigator behavior_server smoother_server waypoint_follower; do
  echo -n "${node}: "
  ros2 lifecycle get "/${node}" 2>&1
done

echo ""
echo "=== TF Check (map -> base_footprint) ==="
timeout 5 ros2 run tf2_ros tf2_echo map base_footprint 2>&1 | head -10

echo ""
echo "=== Topics ==="
ros2 topic list 2>/dev/null | grep -E 'cmd_vel|odom|scan|map|amcl' | head -10

echo ""
echo "=== Nav2 Log (last 30 lines) ==="
tail -30 /home/veni/nav2.log 2>/dev/null
