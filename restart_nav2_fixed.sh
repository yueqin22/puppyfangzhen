#!/bin/bash
# Restart only Nav2 with fixed AMCL config
export HOME=/home/veni
export PATH="/home/veni/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

echo "========================================"
echo "  Step 1: Kill only Nav2 processes"
echo "========================================"
pkill -9 -f 'controller_server' 2>/dev/null || true
pkill -9 -f 'lifecycle_manager' 2>/dev/null || true
pkill -9 -f 'planner_server' 2>/dev/null || true
pkill -9 -f 'bt_navigator' 2>/dev/null || true
pkill -9 -f 'amcl' 2>/dev/null || true
pkill -9 -f 'map_server' 2>/dev/null || true
pkill -9 -f 'behavior_server' 2>/dev/null || true
pkill -9 -f 'waypoint_follower' 2>/dev/null || true
pkill -9 -f 'smoother_server' 2>/dev/null || true
sleep 3
echo "Done"
echo ""

echo "========================================"
echo "  Step 2: Sync config file"
echo "========================================"
cp /mnt/e/puppyfangzhen/src/puppy_nav/config/nav2_params.yaml /home/veni/puppy_ws/src/puppy_nav/config/nav2_params.yaml
sed -i 's/\r$//' /home/veni/puppy_ws/src/puppy_nav/config/nav2_params.yaml
echo "Done"
echo ""

echo "========================================"
echo "  Step 3: Verify Gazebo is still running"
echo "========================================"
if pgrep -f gzserver > /dev/null; then
    echo "OK: Gazebo is running"
else
    echo "FAIL: Gazebo is not running, need to start it"
    cd /home/veni/puppy_ws
    source /home/veni/puppy_ws/install/setup.bash
    nohup ros2 launch puppy_worlds simulation.launch.py gui:=false use_sim_time:=true > /tmp/gazebo.log 2>&1 &
    echo "Waiting 25 seconds for Gazebo..."
    sleep 25
fi
echo ""

echo "========================================"
echo "  Step 4: Start Nav2"
echo "========================================"
cd /home/veni/puppy_ws
source /home/veni/puppy_ws/install/setup.bash
nohup ros2 launch puppy_nav navigation.launch.py use_sim_time:=true > /tmp/nav2.log 2>&1 &
echo "Waiting 25 seconds for Nav2 (with set_initial_pose=true)..."
sleep 25
echo ""

echo "========================================"
echo "  Step 5: Check Nav2 log"
echo "========================================"
echo "Nav2 log (last 30 lines, filtered):"
grep -v 'tick rate' /tmp/nav2.log | tail -30
echo ""

echo "========================================"
echo "  Step 6: Check laser scan"
echo "========================================"
echo "Laser scan ranges (first 10):"
timeout 5 ros2 topic echo /scan --once 2>&1 | grep -A 10 'ranges:' | head -12
echo ""

echo "========================================"
echo "  Step 7: Check TF map->base_footprint"
echo "========================================"
timeout 8 ros2 run tf2_ros tf2_echo map base_footprint 2>&1 | head -15
echo ""

echo "========================================"
echo "  Step 8: Check AMCL initial pose status"
echo "========================================"
ros2 param get /amcl set_initial_pose 2>&1
echo ""

echo "========================================"
echo "  Step 9: Send navigation goal to (1.0, 0.5)"
echo "========================================"
echo "Sending goal..."
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose '{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.5, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}' 2>&1 &
GOAL_PID=$!

echo "Monitoring robot movement for 20 seconds..."
for i in $(seq 1 10); do
    sleep 2
    echo "--- Check $i ($(( i*2 ))s) ---"
    timeout 2 ros2 topic echo /cmd_vel --once 2>&1 | grep -E 'linear' -A 3 | head -5 || echo "No cmd_vel"
done

wait $GOAL_PID 2>/dev/null

echo ""
echo "=== Nav2 log (last 30 lines, filtered) ==="
grep -v 'tick rate' /tmp/nav2.log | tail -30

echo ""
echo "========================================"
echo "  COMPLETE!"
echo "========================================"
