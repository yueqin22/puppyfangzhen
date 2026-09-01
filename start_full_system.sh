#!/bin/bash
set -e

export DISPLAY=:1
export PATH="/home/veni/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
source /opt/ros/humble/setup.bash
source ~/puppy_ws/install/setup.bash
export LD_LIBRARY_PATH="/opt/ros/humble/lib:$LD_LIBRARY_PATH"
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/mnt/e/puppyfangzhen/fastdds_profile.xml
export PYTHONUNBUFFERED=1

SCENE_PATH="/home/veni/puppy_ws/src/puppy_worlds/worlds/home_coppelia.ttt"
COPPELIA_DIR="$HOME/CoppeliaSim"
LOG_DIR="/tmp/puppy_sim"
mkdir -p "$LOG_DIR"

echo "=== [1/3] 启动 CoppeliaSim ==="
pkill -9 -f coppeliaSim 2>/dev/null || true
sleep 2

cd "$COPPELIA_DIR"
nohup ./coppeliaSim.sh -h "$SCENE_PATH" > "$LOG_DIR/coppelia.log" 2>&1 &
COPPELIA_PID=$!
echo "CoppeliaSim PID: $COPPELIA_PID"

echo "等待ZMQ端口就绪..."
for i in $(seq 1 30); do
    if ss -tlnp 2>/dev/null | grep -q ':23000'; then
        echo "ZMQ就绪 (${i}s)"
        break
    fi
    sleep 1
    if ! kill -0 $COPPELIA_PID 2>/dev/null; then
        echo "ERROR: CoppeliaSim已退出!"
        tail -20 "$LOG_DIR/coppelia.log"
        exit 1
    fi
done

sleep 5

echo "检查并启动仿真..."
python3 - <<'PYEOF'
import sys, time
sys.path.insert(0, '/home/veni/CoppeliaSim/programming/zmqRemoteApi/clients/python')
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
client = RemoteAPIClient()
sim = client.getObject('sim')
state = sim.getSimulationState()
print(f"仿真状态: {state}")
if state != sim.simulation_advancing_running:
    print("启动仿真...")
    sim.startSimulation()
    time.sleep(3)
    state = sim.getSimulationState()
    print(f"仿真状态: {state}")
if state == sim.simulation_advancing_running:
    print("OK: 仿真运行中")
else:
    print("ERROR: 仿真未运行")
    sys.exit(1)
PYEOF

echo "=== [2/3] 启动 coppelia_bridge ==="
nohup ros2 run puppy_bringup coppelia_bridge.py --ros-args -p use_sim_time:=false > "$LOG_DIR/bridge.log" 2>&1 &
BRIDGE_PID=$!
echo "Bridge PID: $BRIDGE_PID"
sleep 3

echo "检查bridge话题..."
timeout 10 bash -c 'while ! ros2 topic list 2>/dev/null | grep -q /odom; do sleep 1; done' && echo "Bridge: /odom 话题就绪" || echo "WARN: /odom 未出现"

echo "=== [3/3] 启动ROS2系统 (Nav2 + 智能家居) ==="
nohup ros2 launch puppy_bringup coppelia_system.launch.py > "$LOG_DIR/ros2.log" 2>&1 &
LAUNCH_PID=$!
echo "Launch PID: $LAUNCH_PID"

echo ""
echo "=== 启动完成，等待系统就绪 (20秒) ==="
sleep 20

echo ""
echo "=== 进程状态 ==="
echo "CoppeliaSim: $(pgrep -f coppeliaSim | wc -l) 进程"
echo "Bridge PID: $BRIDGE_PID ($(kill -0 $BRIDGE_PID 2>/dev/null && echo '运行中' || echo '已退出'))"
echo "Launch PID: $LAUNCH_PID ($(kill -0 $LAUNCH_PID 2>/dev/null && echo '运行中' || echo '已退出'))"

echo ""
echo "=== ROS2话题列表 ==="
ros2 topic list 2>/dev/null | head -30

echo ""
echo "=== 节点列表 ==="
ros2 node list 2>/dev/null | head -20

echo ""
echo "=== 日志位置 ==="
echo "CoppeliaSim: $LOG_DIR/coppelia.log"
echo "Bridge: $LOG_DIR/bridge.log"
echo "ROS2: $LOG_DIR/ros2.log"
echo ""
echo "PIDs: coppelia=$COPPELIA_PID bridge=$BRIDGE_PID launch=$LAUNCH_PID"
echo "$COPPELIA_PID $BRIDGE_PID $LAUNCH_PID" > "$LOG_DIR/pids.txt"
echo "启动脚本完成"
