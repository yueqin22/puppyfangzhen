#!/bin/bash
set -e

export PATH="/home/veni/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/veni/fastdds_profile.xml
export PYTHONUNBUFFERED=1
export QT_QPA_PLATFORM=offscreen

source /opt/ros/humble/setup.bash
source ~/puppy_ws/install/setup.bash
export LD_LIBRARY_PATH="/opt/ros/humble/lib:${LD_LIBRARY_PATH:-}"

LOG_DIR="/tmp/puppy_sim"
mkdir -p "$LOG_DIR"
rm -f "$LOG_DIR"/*.log

SCENE="/home/veni/puppy_ws/src/puppy_worlds/worlds/home_coppelia.ttt"
COPPELIA_DIR="/home/veni/CoppeliaSim"

echo "============================================"
echo " Puppy Robot Dog Simulation Startup"
echo "============================================"
echo ""

# Step 1: Kill stale processes
echo "[1/6] Cleaning up stale processes..."
pkill -9 -f coppeliaSim 2>/dev/null || true
pkill -9 -f coppelia_bridge 2>/dev/null || true
pkill -9 -f pythonLauncher 2>/dev/null || true
pkill -9 -f robot_state_publisher 2>/dev/null || true
pkill -9 -f rviz2 2>/dev/null || true
pkill -9 -f lifecycle_manager 2>/dev/null || true
pkill -9 -f amcl 2>/dev/null || true
pkill -9 -f bt_navigator 2>/dev/null || true
pkill -9 -f planner_server 2>/dev/null || true
pkill -9 -f controller_server 2>/dev/null || true
pkill -9 -f behavior_server 2>/dev/null || true
pkill -9 -f battery_simulator 2>/dev/null || true
pkill -9 -f smart_home 2>/dev/null || true
pkill -9 -f fall_detection 2>/dev/null || true
pkill -9 -f emergency_response 2>/dev/null || true
pkill -9 -f security_node 2>/dev/null || true
pkill -9 -f emotion_interaction 2>/dev/null || true
pkill -9 -f task_scheduler 2>/dev/null || true
pkill -9 -f waypoint_follower 2>/dev/null || true
pkill -9 -f smoother_server 2>/dev/null || true
pkill -9 -f map_server 2>/dev/null || true
sleep 3
echo "  Done."

# Step 2: Start CoppeliaSim in headless mode
echo "[2/6] Starting CoppeliaSim (headless mode)..."
cd "$COPPELIA_DIR"
nohup ./coppeliaSim.sh -h -s 1000000000 "$SCENE" > "$LOG_DIR/coppelia.log" 2>&1 &
COPPELIA_PID=$!
echo "  CoppeliaSim PID: $COPPELIA_PID"

# Wait for ZMQ
echo "  Waiting for ZMQ port 23000..."
ZMQ_READY=0
for i in $(seq 1 45); do
    if ss -tlnp 2>/dev/null | grep -q ':23000'; then
        echo "  ZMQ ready after ${i}s."
        ZMQ_READY=1
        break
    fi
    sleep 1
    if ! kill -0 $COPPELIA_PID 2>/dev/null; then
        echo "ERROR: CoppeliaSim exited!"
        tail -30 "$LOG_DIR/coppelia.log"
        exit 1
    fi
done
if [ $ZMQ_READY -eq 0 ]; then
    echo "ERROR: ZMQ port did not open in time!"
    tail -30 "$LOG_DIR/coppelia.log"
    exit 1
fi
sleep 5

# Step 3: Start simulation
echo "[3/6] Starting simulation via ZMQ..."
python3 - <<'PYEOF'
import sys, time
sys.path.insert(0, '/home/veni/CoppeliaSim/programming/zmqRemoteApi/clients/python')
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
client = RemoteAPIClient()
sim = client.getObject('sim')
state = sim.getSimulationState()
if state != sim.simulation_advancing_running:
    print("  Starting simulation...")
    sim.startSimulation()
    time.sleep(3)
state = sim.getSimulationState()
if state == sim.simulation_advancing_running:
    print(f"  Simulation running! (time: {sim.getSimulationTime():.1f}s)")
else:
    print(f"ERROR: Simulation state = {state}")
    sys.exit(1)
client.__del__()
PYEOF
if [ $? -ne 0 ]; then
    echo "ERROR: Failed to start simulation!"
    exit 1
fi
sleep 3

# Step 4: Start ROS2
echo "[4/6] Starting ROS2 system..."
nohup ros2 launch puppy_bringup coppelia_system.launch.py > "$LOG_DIR/ros2.log" 2>&1 &
LAUNCH_PID=$!
echo "  Launch PID: $LAUNCH_PID"

# Step 5: Wait
echo "[5/6] Waiting for initialization (60s)..."
sleep 60

# Step 6: Check
echo ""
echo "[6/6] System Status"
echo "============================================"

echo ""
echo "--- Processes ---"
kill -0 $COPPELIA_PID 2>/dev/null && echo "  CoppeliaSim: RUNNING" || echo "  CoppeliaSim: DEAD"
kill -0 $LAUNCH_PID 2>/dev/null && echo "  ROS2 Launch: RUNNING" || echo "  ROS2 Launch: DEAD"

echo ""
echo "--- Process Deaths ---"
DEATHS=$(grep 'process has died' "$LOG_DIR/ros2.log" 2>/dev/null)
if [ -z "$DEATHS" ]; then
    echo "  None (good!)"
else
    echo "$DEATHS"
fi

echo ""
echo "--- Bridge Output ---"
grep 'coppelia_bridge' "$LOG_DIR/ros2.log" 2>/dev/null | tail -5 || echo "  (waiting...)"

echo ""
echo "--- Topics ---"
for topic in /clock /odom /scan /tf /battery_state /low_battery_alert /fall_detected /emergency/status; do
    if timeout 3 ros2 topic list 2>/dev/null | grep -q "^${topic}$"; then
        echo "  [OK] $topic"
    else
        echo "  [MISSING] $topic"
    fi
done

echo ""
echo "--- Errors (non-TF) ---"
grep -i 'error\|traceback' "$LOG_DIR/ros2.log" 2>/dev/null | grep -v 'Timed out waiting\|Invalid frame' | tail -5 || echo "  None"

echo ""
echo "$COPPELIA_PID $LAUNCH_PID" > "$LOG_DIR/pids.txt"
echo "Simulation is running!"
echo "Logs: $LOG_DIR/"

wait
