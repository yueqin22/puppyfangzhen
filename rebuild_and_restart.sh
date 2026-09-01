#!/bin/bash
# Comprehensive rebuild and restart script for Puppy simulation
set -e

export HOME=/home/veni
export PATH="/home/veni/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

echo "========================================"
echo "  Step 1: Kill all old processes"
echo "========================================"
pkill -9 -f 'ros2' 2>/dev/null || true
pkill -9 -f 'rviz2' 2>/dev/null || true
pkill -9 -f 'gzserver' 2>/dev/null || true
pkill -9 -f 'gzclient' 2>/dev/null || true
pkill -9 -f 'controller_server' 2>/dev/null || true
pkill -9 -f 'lifecycle_manager' 2>/dev/null || true
pkill -9 -f 'planner_server' 2>/dev/null || true
pkill -9 -f 'bt_navigator' 2>/dev/null || true
pkill -9 -f 'amcl' 2>/dev/null || true
pkill -9 -f 'map_server' 2>/dev/null || true
pkill -9 -f 'behavior_server' 2>/dev/null || true
pkill -9 -f 'robot_state_publisher' 2>/dev/null || true
pkill -9 -f 'waypoint_follower' 2>/dev/null || true
pkill -9 -f 'smoother_server' 2>/dev/null || true
pkill -9 -f 'static_transform_publisher' 2>/dev/null || true
pkill -9 x11vnc 2>/dev/null || true
pkill -9 Xvfb 2>/dev/null || true
sleep 3
echo "Done killing processes"
echo ""

echo "========================================"
echo "  Step 2: Sync source files from Windows"
echo "========================================"
cp -r /mnt/e/puppyfangzhen/src/* /home/veni/puppy_ws/src/
echo "Done syncing"
echo ""

echo "========================================"
echo "  Step 3: Fix CRLF line endings"
echo "========================================"
find /home/veni/puppy_ws/src -name '*.yaml' -exec sed -i 's/\r$//' {} \;
find /home/veni/puppy_ws/src -name '*.py' -exec sed -i 's/\r$//' {} \;
find /home/veni/puppy_ws/src -name '*.xacro' -exec sed -i 's/\r$//' {} \;
find /home/veni/puppy_ws/src -name '*.urdf' -exec sed -i 's/\r$//' {} \;
find /home/veni/puppy_ws/src -name '*.world' -exec sed -i 's/\r$//' {} \;
find /home/veni/puppy_ws/src -name '*.rviz' -exec sed -i 's/\r$//' {} \;
echo "Done fixing line endings"
echo ""

echo "========================================"
echo "  Step 4: Rebuild workspace"
echo "========================================"
cd /home/veni/puppy_ws
colcon build --symlink-install --packages-select puppy_nav puppy_worlds puppy_description puppy_bringup puppy_localization 2>&1 | tail -20
echo "Done building"
echo ""

echo "========================================"
echo "  Step 5: Verify map files"
echo "========================================"
source /home/veni/puppy_ws/install/setup.bash
MAP_FILE=/home/veni/puppy_ws/install/puppy_nav/share/puppy_nav/maps/home_map.yaml
echo "Map file: $MAP_FILE"
ls -la "$MAP_FILE"
echo "Map YAML content:"
cat "$MAP_FILE"
echo ""
echo "Map YAML hex (first 100 bytes):"
head -c 100 "$MAP_FILE" | xxd
echo ""
echo "PGM file:"
ls -la /home/veni/puppy_ws/install/puppy_nav/share/puppy_nav/maps/home_map.pgm
echo "PGM header:"
head -c 20 /home/veni/puppy_ws/install/puppy_nav/share/puppy_nav/maps/home_map.pgm | xxd
echo ""

echo "========================================"
echo "  Step 6: Start Gazebo simulation"
echo "========================================"
cd /home/veni/puppy_ws
nohup ros2 launch puppy_worlds simulation.launch.py gui:=false use_sim_time:=true > /tmp/gazebo.log 2>&1 &
GAZEBO_PID=$!
echo "Gazebo PID: $GAZEBO_PID"
echo "Waiting 25 seconds for Gazebo to fully start..."
sleep 25
if pgrep -f gzserver > /dev/null; then
    echo "OK: Gazebo is running"
else
    echo "FAIL: Gazebo failed to start"
    tail -30 /tmp/gazebo.log
    exit 1
fi
echo ""

echo "========================================"
echo "  Step 7: Verify Gazebo topics"
echo "========================================"
echo "ROS2 topics:"
ros2 topic list
echo ""
echo "ROS2 services:"
ros2 service list | head -20
echo ""
echo "TF frames:"
ros2 run tf2_tools view_frames > /dev/null 2>&1 || true
timeout 3 ros2 topic echo /tf_static --once 2>&1 | head -20 || true
echo ""

echo "========================================"
echo "  Step 8: Start Nav2 navigation"
echo "========================================"
cd /home/veni/puppy_ws
nohup ros2 launch puppy_nav navigation.launch.py use_sim_time:=true > /tmp/nav2.log 2>&1 &
NAV2_PID=$!
echo "Nav2 PID: $NAV2_PID"
echo "Waiting 20 seconds for Nav2 to fully start..."
sleep 20
echo ""
echo "Nav2 log (last 40 lines):"
tail -40 /tmp/nav2.log
echo ""

echo "========================================"
echo "  Step 9: Verify Nav2 nodes are active"
echo "========================================"
echo "Lifecycle nodes status:"
ros2 lifecycle list map_server 2>&1 || echo "map_server not found"
echo ""
ros2 lifecycle list amcl 2>&1 || echo "amcl not found"
echo ""
ros2 lifecycle list bt_navigator 2>&1 || echo "bt_navigator not found"
echo ""

echo "========================================"
echo "  Step 10: Check all topics"
echo "========================================"
ros2 topic list
echo ""

echo "========================================"
echo "  Step 11: Check /map topic"
echo "========================================"
timeout 5 ros2 topic echo /map --once 2>&1 | head -20 || echo "No /map topic"
echo ""

echo "========================================"
echo "  Step 12: Check /scan topic"
echo "========================================"
timeout 5 ros2 topic echo /scan --once 2>&1 | head -20 || echo "No /scan topic"
echo ""

echo "========================================"
echo "  Step 13: Check /odom topic"
echo "========================================"
timeout 5 ros2 topic echo /odom --once 2>&1 | head -20 || echo "No /odom topic"
echo ""

echo "========================================"
echo "  COMPLETE! Check results above."
echo "========================================"
