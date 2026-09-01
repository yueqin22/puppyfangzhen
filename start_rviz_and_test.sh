#!/bin/bash
# Start RViz + VNC and test navigation
export HOME=/home/veni
export PATH="/home/veni/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
source /opt/ros/humble/setup.bash
source /home/veni/puppy_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

echo "========================================"
echo "  Step 1: Start Xvfb + x11vnc"
echo "========================================"
# Kill old VNC/Xvfb
pkill -9 x11vnc 2>/dev/null || true
pkill -9 Xvfb 2>/dev/null || true
sleep 1

# Start Xvfb
export DISPLAY=:1
Xvfb :1 -screen 0 1920x1080x24 &
sleep 2

# Start x11vnc
x11vnc -display :1 -forever -shared -rfbport 5900 -nopw -bg -o /tmp/x11vnc.log
sleep 2
echo "VNC started on port 5900"
echo ""

echo "========================================"
echo "  Step 2: Start RViz2"
echo "========================================"
RVIZ_CONFIG=/home/veni/puppy_ws/install/puppy_bringup/share/puppy_bringup/rviz/navigation.rviz
nohup rviz2 -d "$RVIZ_CONFIG" --ros-args -p use_sim_time:=true > /tmp/rviz.log 2>&1 &
RVIZ_PID=$!
echo "RViz PID: $RVIZ_PID"
sleep 5
if pgrep -f rviz2 > /dev/null; then
    echo "OK: RViz is running"
else
    echo "FAIL: RViz failed to start"
    tail -20 /tmp/rviz.log
fi
echo ""

echo "========================================"
echo "  Step 3: Set initial pose (again to be sure)"
echo "========================================"
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891909122467]}}' 2>&1
sleep 3
echo ""

echo "========================================"
echo "  Step 4: Send test navigation goal"
echo "========================================"
echo "Sending goal to (1.5, 0.0, 0.0)..."
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose '{pose: {header: {frame_id: map}, pose: {position: {x: 1.5, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}' 2>&1 &
GOAL_PID=$!
echo "Goal sent, waiting 15 seconds to see if robot moves..."
sleep 15

echo ""
echo "=== Check /cmd_vel (robot velocity) ==="
timeout 3 ros2 topic echo /cmd_vel --once 2>&1

echo ""
echo "=== Check robot position ==="
timeout 3 ros2 topic echo /odom --once 2>&1 | head -15

echo ""
echo "========================================"
echo "  Step 5: Check Nav2 log for errors"
echo "========================================"
tail -30 /tmp/nav2.log

echo ""
echo "========================================"
echo "  COMPLETE! VNC available at localhost:5900"
echo "========================================"
