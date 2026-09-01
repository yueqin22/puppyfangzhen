#!/bin/bash
# Complete restart with VNC fix and robot stability fix

echo "========================================"
echo "  Puppy Simulation - Full Restart v2"
echo "========================================"

# Step 1: Kill all processes
echo ""
echo "[1/7] Killing all processes..."
pkill -f 'nav2' 2>/dev/null || true
pkill -f 'ros2 launch' 2>/dev/null || true
pkill -f 'gzserver' 2>/dev/null || true
pkill -f 'gzclient' 2>/dev/null || true
pkill -f 'rviz2' 2>/dev/null || true
pkill -f 'x11vnc' 2>/dev/null || true
pkill -f 'Xvfb' 2>/dev/null || true
pkill -f 'robot_state_publisher' 2>/dev/null || true
sleep 3
echo "  Done."

# Step 2: Sync files from Windows
echo ""
echo "[2/7] Syncing files from Windows..."
cp -r /mnt/e/puppyfangzhen/src/* /home/veni/puppy_ws/src/
find /home/veni/puppy_ws/src -type f \( -name "*.yaml" -o -name "*.py" -o -name "*.xacro" -o -name "*.urdf" -o -name "*.world" -o -name "*.rviz" -o -name "*.xml" -o -name "*.sh" \) -exec sed -i 's/\r$//' {} +
echo "  Done."

# Step 3: Rebuild workspace
echo ""
echo "[3/7] Rebuilding workspace..."
cd /home/veni/puppy_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select puppy_description puppy_nav puppy_worlds puppy_bringup puppy_localization puppy_gait 2>&1 | tail -5
source /home/veni/puppy_ws/install/setup.bash
echo "  Done."

# Step 4: Start Xvfb + VNC (with Wayland fix)
echo ""
echo "[4/7] Starting Xvfb + VNC..."
export DISPLAY=:99
# Kill any existing Xvfb
pkill -f 'Xvfb :99' 2>/dev/null || true
sleep 1
# Start Xvfb
Xvfb :99 -screen 0 1920x1080x24 &
sleep 2
# Fix Wayland detection: unset WAYLAND_DISPLAY and set XDG_SESSION_TYPE
export WAYLAND_DISPLAY=""
export XDG_SESSION_TYPE=x11
# Start x11vnc
x11vnc -display :99 -forever -nopw -rfbport 5900 -shared &
sleep 2
# Verify VNC is running
if pgrep -x x11vnc > /dev/null; then
    echo "  VNC running on port 5900"
else
    echo "  WARNING: x11vnc failed, trying Xvnc..."
    # Try Xvnc as fallback
    vncserver :99 -geometry 1920x1080 -depth 24 2>/dev/null || true
    export DISPLAY=:99
fi

# Step 5: Start Gazebo (headless)
echo ""
echo "[5/7] Starting Gazebo (headless)..."
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export GAZEBO_MODEL_PATH="/home/veni/puppy_ws/src/puppy_description/models:${GAZEBO_MODEL_PATH}"

nohup ros2 launch puppy_worlds simulation.launch.py gui:=false use_sim_time:=true > /home/veni/gazebo.log 2>&1 &
echo "  Waiting 25s for robot to spawn..."
sleep 25

# Check spawn
if grep -q "Successfully spawned" /home/veni/gazebo.log 2>/dev/null; then
    echo "  Robot spawned successfully!"
else
    echo "  Checking spawn status..."
    grep -i "spawn" /home/veni/gazebo.log 2>/dev/null | tail -3
fi

# Step 6: Start Nav2
echo ""
echo "[6/7] Starting Nav2..."
nohup ros2 launch puppy_nav navigation.launch.py > /home/veni/nav2.log 2>&1 &
echo "  Waiting 30s for all nodes to activate..."
sleep 30

# Check Nav2
echo "  Nav2 lifecycle states:"
for node in map_server amcl planner_server controller_server bt_navigator; do
    state=$(ros2 lifecycle get "/${node}" 2>&1)
    echo "    ${node}: ${state}"
done

# Step 7: Start RViz + Verify
echo ""
echo "[7/7] Starting RViz and verifying..."
RVIZ_CONFIG="/home/veni/puppy_ws/install/puppy_bringup/share/puppy_bringup/rviz/navigation.rviz"
if [ ! -f "$RVIZ_CONFIG" ]; then
    RVIZ_CONFIG="/home/veni/puppy_ws/install/puppy_nav/share/puppy_nav/rviz/navigation.rviz"
fi
nohup rviz2 -d "$RVIZ_CONFIG" --ros-args -p use_sim_time:=true > /home/veni/rviz.log 2>&1 &
sleep 5

# Verify system
echo ""
echo "=== Verification ==="
echo "--- /odom (robot position) ---"
timeout 5 ros2 topic echo /odom --once 2>&1 | grep -A 5 "position:" | head -6

echo ""
echo "--- /scan (first 5 ranges) ---"
timeout 5 ros2 topic echo /scan --once 2>&1 | grep -A 5 "ranges:" | head -6

echo ""
echo "--- TF: map -> odom ---"
timeout 5 ros2 run tf2_ros tf2_echo map odom 2>&1 | tail -5

echo ""
echo "--- TF: odom -> base_footprint ---"
timeout 5 ros2 run tf2_ros tf2_echo odom base_footprint 2>&1 | tail -5

echo ""
echo "--- Processes ---"
ps aux | grep -E 'gzserver|rviz2|x11vnc|Xvfb' | grep -v grep | awk '{print $2, $11, $12, $13}' | head -10

echo ""
echo "========================================"
echo "  Restart Complete!"
echo "  VNC: localhost:5900"
echo "  Gazebo log: /home/veni/gazebo.log"
echo "  Nav2 log: /home/veni/nav2.log"
echo "========================================"
