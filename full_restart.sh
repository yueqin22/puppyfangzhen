#!/bin/bash
# Complete restart script for Puppy simulation
# Kills everything, syncs files, rebuilds, starts fresh

set -e

echo "========================================"
echo "  Puppy Simulation Complete Restart"
echo "========================================"

# Step 1: Kill all processes
echo ""
echo "[1/8] Killing all processes..."
pkill -f 'nav2' 2>/dev/null || true
pkill -f 'ros2 launch' 2>/dev/null || true
pkill -f 'gzserver' 2>/dev/null || true
pkill -f 'gzclient' 2>/dev/null || true
pkill -f 'rviz2' 2>/dev/null || true
pkill -f 'x11vnc' 2>/dev/null || true
pkill -f 'Xvfb' 2>/dev/null || true
pkill -f 'robot_state_publisher' 2>/dev/null || true
sleep 3
echo "  Done. Remaining processes:"
ps aux | grep -E 'gzserver|nav2|rviz|x11vnc|Xvfb' | grep -v grep | wc -l

# Step 2: Sync files from Windows
echo ""
echo "[2/8] Syncing files from Windows..."
cp -r /mnt/e/puppyfangzhen/src/* /home/veni/puppy_ws/src/
# Fix CRLF line endings
find /home/veni/puppy_ws/src -type f \( -name "*.yaml" -o -name "*.py" -o -name "*.xacro" -o -name "*.urdf" -o -name "*.world" -o -name "*.rviz" -o -name "*.xml" -o -name "*.sh" \) -exec sed -i 's/\r$//' {} +
echo "  Done."

# Step 3: Rebuild workspace
echo ""
echo "[3/8] Rebuilding workspace..."
cd /home/veni/puppy_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select puppy_description puppy_nav puppy_worlds puppy_bringup puppy_localization puppy_gait 2>&1 | tail -5
source /home/veni/puppy_ws/install/setup.bash
echo "  Done."

# Step 4: Start Xvfb + VNC
echo ""
echo "[4/8] Starting Xvfb + VNC..."
export DISPLAY=:99
Xvfb :99 -screen 0 1920x1080x24 &
sleep 2
x11vnc -display :99 -forever -nopw -rfbport 5900 &
sleep 2
echo "  VNC running on port 5900"

# Step 5: Start Gazebo (headless)
echo ""
echo "[5/8] Starting Gazebo (headless)..."
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export GAZEBO_MODEL_PATH="/home/veni/puppy_ws/src/puppy_description/models:${GAZEBO_MODEL_PATH}"

nohup ros2 launch puppy_worlds simulation.launch.py gui:=false use_sim_time:=true > /home/veni/gazebo.log 2>&1 &
echo "  Gazebo starting... waiting 20s for robot to spawn"
sleep 20

# Check if robot spawned
if grep -q "Spawn status: 0" /home/veni/gazebo.log 2>/dev/null || grep -q "Successfully spawned entity" /home/veni/gazebo.log 2>/dev/null; then
    echo "  Robot spawned successfully!"
else
    echo "  WARNING: Robot spawn status unclear, checking..."
    grep -i "spawn" /home/veni/gazebo.log 2>/dev/null | tail -5
fi

# Step 6: Start Nav2
echo ""
echo "[6/8] Starting Nav2..."
nohup ros2 launch puppy_nav navigation.launch.py > /home/veni/nav2.log 2>&1 &
echo "  Nav2 starting... waiting 25s for all nodes to activate"
sleep 25

# Check Nav2 lifecycle
echo "  Checking Nav2 lifecycle states..."
for node in map_server amcl planner_server controller_server bt_navigator; do
    state=$(ros2 lifecycle get "/${node}" 2>&1)
    echo "    ${node}: ${state}"
done

# Step 7: Start RViz
echo ""
echo "[7/8] Starting RViz..."
export DISPLAY=:99
RVIZ_CONFIG="/home/veni/puppy_ws/install/puppy_bringup/share/puppy_bringup/rviz/navigation.rviz"
if [ ! -f "$RVIZ_CONFIG" ]; then
    RVIZ_CONFIG="/home/veni/puppy_ws/install/puppy_nav/share/puppy_nav/rviz/navigation.rviz"
fi
nohup rviz2 -d "$RVIZ_CONFIG" --ros-args -p use_sim_time:=true > /home/veni/rviz.log 2>&1 &
sleep 5
echo "  RViz started."

# Step 8: Verify system
echo ""
echo "[8/8] Verifying system..."
echo "  === /scan topic ==="
timeout 3 ros2 topic echo /scan --once 2>&1 | head -5

echo ""
echo "  === /odom topic ==="
timeout 3 ros2 topic echo /odom --once 2>&1 | head -12

echo ""
echo "  === TF: map -> odom ==="
timeout 5 ros2 run tf2_ros tf2_echo map odom 2>&1 | head -8

echo ""
echo "  === TF: odom -> base_footprint ==="
timeout 5 ros2 run tf2_ros tf2_echo odom base_footprint 2>&1 | head -8

echo ""
echo "========================================"
echo "  Restart Complete!"
echo "  VNC: localhost:5900"
echo "  Gazebo log: /home/veni/gazebo.log"
echo "  Nav2 log: /home/veni/nav2.log"
echo "  RViz log: /home/veni/rviz.log"
echo "========================================"
