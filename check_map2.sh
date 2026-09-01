#!/bin/bash
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
source ~/puppy_ws/install/setup.bash

echo "=== Try to get map (5s timeout) ==="
timeout 5 ros2 topic echo /map --once --qos-reliability reliable --qos-durability transient_local 2>&1 | head -20
echo "EXIT: $?"

echo "=== Global costmap params ==="
ros2 param get /global_costmap/global_costmap width 2>&1
ros2 param get /global_costmap/global_costmap height 2>&1
ros2 param get /global_costmap/global_costmap resolution 2>&1
ros2 param get /global_costmap/global_costmap global_frame 2>&1
ros2 param get /global_costmap/global_costmap rolling_window 2>&1

echo "=== Static layer params ==="
ros2 param get /global_costmap/global_costmap plugins 2>&1

echo "=== Map topic QoS ==="
ros2 topic info /map -v 2>&1
