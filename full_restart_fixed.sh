#!/bin/bash
# Complete restart with fixed laser and costmap configuration
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
echo "Done"
echo ""

echo "========================================"
echo "  Step 2: Sync and fix files"
echo "========================================"
cp -r /mnt/e/puppyfangzhen/src/* /home/veni/puppy_ws/src/
find /home/veni/puppy_ws/src -name '*.yaml' -exec sed -i 's/\r$//' {} \;
find /home/veni/puppy_ws/src -name '*.py' -exec sed -i 's/\r$//' {} \;
find /home/veni/puppy_ws/src -name '*.xacro' -exec sed -i 's/\r$//' {} \;
find /home/veni/puppy_ws/src -name '*.urdf' -exec sed -i 's/\r$//' {} \;
find /home/veni/puppy_ws/src -name '*.world' -exec sed -i 's/\r$//' {} \;
find /home/veni/puppy_ws/src -name '*.rviz' -exec sed -i 's/\r$//' {} \;
echo "Done"
echo ""

echo "========================================"
echo "  Step 3: Rebuild workspace"
echo "========================================"
cd /home/veni/puppy_ws
colcon build --symlink-install --packages-select puppy_nav puppy_worlds puppy_description puppy_bringup puppy_localization 2>&1 | tail -10
source /home/veni/puppy_ws/install/setup.bash
echo "Done"
echo ""

echo "========================================"
echo "  Step 4: Start Gazebo"
echo "========================================"
cd /home/veni/puppy_ws
nohup ros2 launch puppy_worlds simulation.launch.py gui:=false use_sim_time:=true > /tmp/gazebo.log 2>&1 &
echo "Waiting 25 seconds for Gazebo..."
sleep 25
if pgrep -f gzserver > /dev/null; then
    echo "OK: Gazebo is running"
else
    echo "FAIL: Gazebo failed"
    tail -20 /tmp/gazebo.log
    exit 1
fi
echo ""

echo "========================================"
echo "  Step 5: Start Nav2"
echo "========================================"
cd /home/veni/puppy_ws
nohup ros2 launch puppy_nav navigation.launch.py use_sim_time:=true > /tmp/nav2.log 2>&1 &
echo "Waiting 20 seconds for Nav2..."
sleep 20
echo "Nav2 log (last 15 lines, filtered):"
grep -v 'tick rate' /tmp/nav2.log | tail -15
echo ""

echo "========================================"
echo "  Step 6: Start VNC + RViz"
echo "========================================"
export DISPLAY=:1
Xvfb :1 -screen 0 1920x1080x24 &
sleep 2
x11vnc -display :1 -forever -shared -rfbport 5900 -nopw -bg -o /tmp/x11vnc.log
sleep 2
RVIZ_CONFIG=/home/veni/puppy_ws/install/puppy_bringup/share/puppy_bringup/rviz/navigation.rviz
nohup rviz2 -d "$RVIZ_CONFIG" --ros-args -p use_sim_time:=true > /tmp/rviz.log 2>&1 &
sleep 5
echo "VNC on port 5900, RViz started"
echo ""

echo "========================================"
echo "  Step 7: Set initial pose"
echo "========================================"
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467]}}' 2>&1
sleep 3
echo ""

echo "========================================"
echo "  Step 8: Verify laser scan (should NOT see self)"
echo "========================================"
echo "Laser scan ranges (first 10):"
timeout 5 ros2 topic echo /scan --once 2>&1 | grep -A 10 'ranges:' | head -12
echo ""

echo "========================================"
echo "  Step 9: Check TF map->base_footprint"
echo "========================================"
timeout 5 ros2 run tf2_ros tf2_echo map base_footprint 2>&1 | head -10
echo ""

echo "========================================"
echo "  Step 10: Send test navigation goal to (1.0, 0.5)"
echo "========================================"
echo "Sending goal..."
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose '{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.5, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}' 2>&1 &
GOAL_PID=$!

echo "Monitoring robot movement for 20 seconds..."
for i in $(seq 1 10); do
    sleep 2
    echo "--- Check $i ($(( i*2 ))s) ---"
    timeout 2 ros2 topic echo /cmd_vel --once 2>&1 | grep -E 'linear|x:|y:|z:' | head -5 || echo "No cmd_vel"
done

echo ""
echo "=== Nav2 log (last 20 lines, filtered) ==="
grep -v 'tick rate' /tmp/nav2.log | tail -20

echo ""
echo "========================================"
echo "  COMPLETE! VNC at localhost:5900"
echo "========================================"
