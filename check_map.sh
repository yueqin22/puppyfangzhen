#!/bin/bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
source ~/puppy_ws/install/setup.bash

echo "=== Lifecycle states ==="
for node in map_server amcl controller_server planner_server behavior_server bt_navigator; do
  echo -n "$node: "
  ros2 lifecycle get /$node 2>&1
done

echo "=== Map server param ==="
ros2 param get /map_server yaml_filename 2>&1

echo "=== Map file check ==="
ls -la ~/puppy_ws/src/puppy_nav/maps/home_map.* 2>&1
echo "--- YAML content ---"
cat ~/puppy_ws/src/puppy_nav/maps/home_map.yaml 2>&1

echo "=== Installed map file check ==="
ls -la ~/puppy_ws/install/puppy_nav/share/puppy_nav/maps/home_map.* 2>&1
