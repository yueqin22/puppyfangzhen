#!/bin/bash
# 统一启动脚本 - 在同一进程组中启动所有组件
LOG_DIR="/tmp/puppy_sim"
mkdir -p "$LOG_DIR"

echo "============================================"
echo "  Puppy 仿真系统统一启动"
echo "============================================"

# ===== 1. 清理旧进程 =====
echo "[1/6] 清理旧进程..."
pkill -9 -f coppeliaSim 2>/dev/null || true
pkill -9 -f pythonLauncher 2>/dev/null || true
pkill -9 -f 'ros2 launch' 2>/dev/null || true
pkill -9 -f coppelia_bridge 2>/dev/null || true
pkill -9 -f battery_simulator 2>/dev/null || true
pkill -9 -f emergency_response 2>/dev/null || true
pkill -9 -f fall_detection 2>/dev/null || true
pkill -9 Xvfb 2>/dev/null || true
sleep 2

# ===== 2. 使用 WSLg 显示 (无需Xvfb/VNC) =====
echo "[2/7] 使用 WSLg 显示..."
export DISPLAY=:0
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe

# ===== 3. 启动 CoppeliaSim (WSLg, setsid 确保独立运行) =====
echo "[3/7] 启动 CoppeliaSim..."
cd /home/veni/CoppeliaSim
setsid ./coppeliaSim.sh -s 1000000000 /home/veni/puppy_ws/src/puppy_worlds/worlds/home_coppelia.ttt > "$LOG_DIR/coppelia.log" 2>&1 &
COPPELIA_PID=$!
disown
echo "  CoppeliaSim PID=$COPPELIA_PID"

echo "  等待 ZMQ 端口 (最多45秒)..."
ZMQ_OK=0
for i in $(seq 1 45); do
    sleep 1
    if ss -tlnp 2>/dev/null | grep -q ':23000'; then
        echo "  ZMQ 就绪! (${i}s)"
        ZMQ_OK=1
        break
    fi
    if ! kill -0 $COPPELIA_PID 2>/dev/null; then
        echo "  ERROR: CoppeliaSim 退出!"
        tail -15 "$LOG_DIR/coppelia.log"
        exit 1
    fi
done

if [ "$ZMQ_OK" = "0" ]; then
    echo "  ERROR: ZMQ 未启动"
    tail -15 "$LOG_DIR/coppelia.log"
    exit 1
fi

# ===== 4. 验证仿真已自动启动 =====
echo "[4/6] 验证仿真状态..."
sleep 3
timeout 10 python3 -c "
import sys
sys.path.insert(0, '/home/veni/CoppeliaSim/programming/zmqRemoteApi/clients/python')
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
client = RemoteAPIClient()
sim = client.getObject('sim')
st = sim.getSimulationTime()
state = sim.getSimulationState()
print(f'  仿真状态: State={state} Time={st:.2f}s')
" 2>&1 || echo "  WARNING: ZMQ验证失败，继续..."

# ===== 5. 启动 ROS2 系统 (setsid 确保独立运行) =====
echo "[5/6] 启动 ROS2 系统..."
export PATH="/home/veni/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/veni/fastdds_profile.xml
export PYTHONUNBUFFERED=1

source /opt/ros/humble/setup.bash
source /home/veni/puppy_ws/install/setup.bash
export LD_LIBRARY_PATH="/opt/ros/humble/lib:${LD_LIBRARY_PATH:-}"

setsid bash -c 'source /opt/ros/humble/setup.bash; source /home/veni/puppy_ws/install/setup.bash; export LD_LIBRARY_PATH="/opt/ros/humble/lib:${LD_LIBRARY_PATH:-}"; export ROS_DOMAIN_ID=0; export ROS_LOCALHOST_ONLY=1; export RMW_IMPLEMENTATION=rmw_fastrtps_cpp; export FASTRTPS_DEFAULT_PROFILES_FILE=/home/veni/fastdds_profile.xml; export PYTHONUNBUFFERED=1; ros2 launch puppy_bringup coppelia_system.launch.py' > "$LOG_DIR/ros2.log" 2>&1 &
LAUNCH_PID=$!
disown
echo "  ROS2 Launch PID=$LAUNCH_PID"
echo $LAUNCH_PID > "$LOG_DIR/launch_pid.txt"

# ===== 6. 等待节点初始化 =====
echo "[6/7] 等待节点初始化 (45秒)..."
sleep 45

# ===== 7. 重启Bridge（launch启动的bridge会卡在ZMQ，需手动重启） =====
echo "[7/7] 重启 Bridge 节点..."
pkill -9 -f 'coppelia_bridge.py' 2>/dev/null || true
sleep 10  # 等待ZMQ连接释放
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null  # 清理SHM

setsid bash -c 'source /opt/ros/humble/setup.bash; source /home/veni/puppy_ws/install/setup.bash; export LD_LIBRARY_PATH="/opt/ros/humble/lib:${LD_LIBRARY_PATH:-}"; export ROS_DOMAIN_ID=0; export ROS_LOCALHOST_ONLY=1; export RMW_IMPLEMENTATION=rmw_fastrtps_cpp; export FASTRTPS_DEFAULT_PROFILES_FILE=/home/veni/fastdds_profile.xml; export PYTHONUNBUFFERED=1; export DISPLAY=:0; python3 /home/veni/puppy_ws/install/puppy_bringup/lib/puppy_bringup/coppelia_bridge.py --ros-args -r __node:=coppelia_bridge' > "$LOG_DIR/bridge.log" 2>&1 &
BRIDGE_PID=$!
disown
echo "  Bridge PID=$BRIDGE_PID"
echo "  等待Bridge初始化 (80秒)..."
sleep 80

echo ""
echo "============================================"
echo "  状态检查"
echo "============================================"

echo ""
echo "--- 进程状态 ---"
echo "CoppeliaSim: $(kill -0 $COPPELIA_PID 2>/dev/null && echo '运行中' || echo '已退出')"
echo "ROS2 Launch: $(kill -0 $LAUNCH_PID 2>/dev/null && echo '运行中' || echo '已退出')"
echo "Bridge:      $(kill -0 $BRIDGE_PID 2>/dev/null && echo '运行中' || echo '已退出')"

echo ""
echo "--- ZMQ 端口 ---"
ss -tlnp 2>/dev/null | grep ':23000' && echo "  ZMQ: 正常" || echo "  ZMQ: 未监听!"

echo ""
echo "--- 进程死亡记录 ---"
grep 'process has died' "$LOG_DIR/ros2.log" 2>/dev/null || echo "  无"

echo ""
echo "--- Bridge 日志 ---"
tail -5 "$LOG_DIR/bridge.log" 2>/dev/null

echo ""
echo "--- Battery 日志 ---"
grep -i 'battery_sim' "$LOG_DIR/ros2.log" 2>/dev/null | tail -5

echo ""
echo "--- Emergency 日志 ---"
grep -i 'emergency' "$LOG_DIR/ros2.log" 2>/dev/null | tail -3

echo ""
echo "--- 错误信息 ---"
grep -iE 'error|traceback|exception' "$LOG_DIR/ros2.log" 2>/dev/null | grep -v 'Timed out\|Invalid frame\|TF' | tail -8 || echo "  无"

echo ""
echo "--- 话题列表 ---"
timeout 10 ros2 topic list 2>/dev/null | head -20

echo ""
echo "============================================"
echo "  启动完成!"
echo "  日志文件: $LOG_DIR/"
echo "============================================"
