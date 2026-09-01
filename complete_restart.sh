#!/bin/bash
# Complete restart - Gazebo + Nav2 + RViz + VNC
export HOME=/home/veni
export PATH="/home/veni/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

echo "========================================"
echo "  COMPLETE RESTART - Fresh state"
echo "========================================"

echo "=== Kill ALL processes ==="
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
sleep 5
echo "Done"
echo ""

echo "=== Sync files ==="
cp -r /mnt/e/puppyfangzhen/src/* /home/veni/puppy_ws/src/
find /home/veni/puppy_ws/src -name '*.yaml' -exec sed -i 's/\r$//' {} \;
find /home/veni/puppy_ws/src -name '*.py' -exec sed -i 's/\r$//' {} \;
find /home/veni/puppy_ws/src -name '*.xacro' -exec sed -i 's/\r$//' {} \;
find /home/veni/puppy_ws/src -name '*.world' -exec sed -i 's/\r$//' {} \;
find /home/veni/puppy_ws/src -name '*.rviz' -exec sed -i 's/\r$//' {} \;
echo "Done"
echo ""

echo "=== Start Gazebo ==="
cd /home/veni/puppy_ws
source /home/veni/puppy_ws/install/setup.bash
nohup ros2 launch puppy_worlds simulation.launch.py gui:=false use_sim_time:=true > /tmp/gazebo.log 2>&1 &
echo "Waiting 30 seconds for Gazebo..."
sleep 30
if pgrep -f gzserver > /dev/null; then
    echo "OK: Gazebo is running"
else
    echo "FAIL: Gazebo failed"
    exit 1
fi
echo ""

echo "=== Start Nav2 ==="
nohup ros2 launch puppy_nav navigation.launch.py use_sim_time:=true > /tmp/nav2.log 2>&1 &
echo "Waiting 25 seconds for Nav2..."
sleep 25
echo ""

echo "=== Start VNC + RViz ==="
export DISPLAY=:1
Xvfb :1 -screen 0 1920x1080x24 &
sleep 2
x11vnc -display :1 -forever -shared -rfbport 5900 -nopw -bg -o /tmp/x11vnc.log
sleep 2
RVIZ_CONFIG=/home/veni/puppy_ws/install/puppy_bringup/share/puppy_bringup/rviz/navigation.rviz
nohup rviz2 -d "$RVIZ_CONFIG" --ros-args -p use_sim_time:=true > /tmp/rviz.log 2>&1 &
sleep 5
echo "VNC on port 5900"
echo ""

echo "=== Verify TF chain ==="
echo "map->odom:"
timeout 5 ros2 run tf2_ros tf2_echo map odom 2>&1 | grep -E 'Translation|Rotation' | head -3
echo ""
echo "odom->base_footprint:"
timeout 5 ros2 run tf2_ros tf2_echo odom base_footprint 2>&1 | grep -E 'Translation|Rotation' | head -3
echo ""
echo "map->base_footprint:"
timeout 5 ros2 run tf2_ros tf2_echo map base_footprint 2>&1 | grep -E 'Translation|Rotation' | head -3
echo ""

echo "=== Check laser scan (should be clean now) ==="
timeout 5 ros2 topic echo /scan --once 2>&1 | grep -A 5 'ranges:' | head -7
echo ""

echo "=== Send navigation goal to (1.0, 0.0) ==="
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped '{header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}' 2>&1
echo "Goal sent, monitoring for 15 seconds..."

for i in $(seq 1 8); do
    sleep 2
    echo "--- $((i*2))s ---"
    timeout 2 ros2 topic echo /cmd_vel --once 2>&1 | grep -A 3 'linear:' | head -4
    timeout 2 ros2 topic echo /odom --once 2>&1 | grep -A 3 'position:' | head -4
done

echo ""
echo "=== Nav2 log (last 20 lines) ==="
grep -v 'tick rate' /tmp/nav2.log | tail -20

echo ""
echo "=== DONE ==="
