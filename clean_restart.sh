#!/bin/bash
# Complete clean restart with thorough diagnostics

echo "========================================"
echo "  COMPLETE CLEAN RESTART + DIAGNOSTICS"
echo "========================================"

# Step 1: Kill EVERYTHING
echo ""
echo "[1/7] Killing ALL processes..."
pkill -9 -f 'nav2' 2>/dev/null || true
pkill -9 -f 'ros2' 2>/dev/null || true
pkill -9 -f 'gzserver' 2>/dev/null || true
pkill -9 -f 'gzclient' 2>/dev/null || true
pkill -9 -f 'rviz2' 2>/dev/null || true
pkill -9 -f 'x11vnc' 2>/dev/null || true
pkill -9 -f 'Xvfb' 2>/dev/null || true
pkill -9 -f 'Xvnc' 2>/dev/null || true
pkill -9 -f 'robot_state_publisher' 2>/dev/null || true
pkill -9 -f 'send_goal' 2>/dev/null || true
pkill -9 -f 'patrol' 2>/dev/null || true
pkill -9 -f 'python3' 2>/dev/null || true
sleep 3

# Also kill any remaining gazebo/ros processes
killall -9 gzserver gzclient rviz2 2>/dev/null || true
sleep 2
echo "  Remaining ROS processes:"
ps aux | grep -E 'gzserver|nav2|rviz|ros2' | grep -v grep | wc -l

# Step 2: Sync files
echo ""
echo "[2/7] Syncing files..."
cp -r /mnt/e/puppyfangzhen/src/* /home/veni/puppy_ws/src/
find /home/veni/puppy_ws/src -type f \( -name "*.yaml" -o -name "*.py" -o -name "*.xacro" -o -name "*.urdf" -o -name "*.world" -o -name "*.rviz" -o -name "*.xml" -o -name "*.sh" \) -exec sed -i 's/\r$//' {} +
echo "  Done."

# Step 3: Rebuild
echo ""
echo "[3/7] Rebuilding..."
cd /home/veni/puppy_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select puppy_description puppy_nav puppy_worlds puppy_bringup puppy_localization puppy_gait 2>&1 | tail -5
source /home/veni/puppy_ws/install/setup.bash

# Step 4: Start Gazebo fresh
echo ""
echo "[4/7] Starting Gazebo..."
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export DISPLAY=:0
export WAYLAND_DISPLAY=wayland-0
export GAZEBO_MODEL_PATH="/home/veni/puppy_ws/src/puppy_description/urdf:${GAZEBO_MODEL_PATH}"

nohup ros2 launch puppy_worlds simulation.launch.py gui:=false use_sim_time:=true > /home/veni/gazebo.log 2>&1 &
echo "  Waiting 30s for robot to spawn and settle..."
sleep 30

echo "  Gazebo spawn status:"
grep -i "spawn\|entity\|model" /home/veni/gazebo.log 2>/dev/null | tail -5

# Step 5: Start Nav2 fresh
echo ""
echo "[5/7] Starting Nav2..."
nohup ros2 launch puppy_nav navigation.launch.py > /home/veni/nav2.log 2>&1 &
echo "  Waiting 35s for all nodes to activate..."
sleep 35

echo "  Lifecycle states:"
for node in map_server amcl planner_server controller_server behavior_server bt_navigator smoother_server waypoint_follower; do
    state=$(ros2 lifecycle get "/${node}" 2>&1)
    echo "    ${node}: ${state}"
done

# Step 6: Comprehensive diagnostics
echo ""
echo "[6/7] Running diagnostics..."

echo ""
echo "=== A. Topic list ==="
ros2 topic list 2>&1 | sort

echo ""
echo "=== B. /odom (robot position) ==="
timeout 3 ros2 topic echo /odom --once 2>&1 | grep -E "x:|y:|z:|w:" | head -10

echo ""
echo "=== C. TF: map -> odom -> base_footprint ==="
timeout 5 ros2 run tf2_ros tf2_echo map base_footprint 2>&1 | grep -E "Translation|Rotation|RPY" | head -6

echo ""
echo "=== D. TF: odom -> base_footprint ==="
timeout 5 ros2 run tf2_ros tf2_echo odom base_footprint 2>&1 | grep -E "Translation|Rotation|RPY" | head -6

echo ""
echo "=== E. /scan (laser data) ==="
timeout 3 ros2 topic echo /scan --once 2>&1 | grep -E "frame_id|angle_min|angle_max|range_min|range_max" | head -5
echo "  First 5 ranges:"
timeout 3 ros2 topic echo /scan --once 2>&1 | grep -A 5 "ranges:" | head -6

echo ""
echo "=== F. /amcl_pose ==="
timeout 3 ros2 topic echo /amcl_pose --once 2>&1 | grep -E "x:|y:|z:|w:" | head -7

echo ""
echo "=== G. /cmd_vel (should be empty - no goals sent) ==="
timeout 3 ros2 topic echo /cmd_vel --once 2>&1 | head -5

echo ""
echo "=== H. Goal topics (should have NO publishers) ==="
echo "  /goal_pose publishers:"
ros2 topic info /goal_pose 2>&1 | head -5
echo "  /navigate_to_pose action servers:"
ros2 action list 2>&1 | grep navigate

echo ""
echo "=== I. Nav2 log - checking for unexpected goals ==="
grep -i "goal\|navigat\|plan\|preempt" /home/veni/nav2.log 2>/dev/null | tail -15

echo ""
echo "=== J. Costmap topics ==="
ros2 topic list 2>&1 | grep costmap

echo ""
echo "=== K. Global costmap metadata ==="
timeout 3 ros2 topic echo /global_costmap/costmap_updates --once 2>&1 | head -3

# Step 7: Summary
echo ""
echo "[7/7] Summary"
echo "========================================"
echo "  System should be IDLE (no goals sent)"
echo "  If /cmd_vel has data, something is wrong"
echo "  If Nav2 log shows goals, something is wrong"
echo "========================================"
echo ""
echo "To send a test goal:"
echo "  python3 /mnt/e/puppyfangzhen/send_goal.py 1.0 0.0 0.0"
echo ""
echo "To start patrol:"
echo "  python3 /mnt/e/puppyfangzhen/patrol.py"
