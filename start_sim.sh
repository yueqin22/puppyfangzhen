#!/bin/bash
# 启动CoppeliaSim + 加载场景 + 启动仿真
export DISPLAY=:1
export PATH="/home/veni/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
source /opt/ros/humble/setup.bash
source ~/puppy_ws/install/setup.bash
export LD_LIBRARY_PATH="/opt/ros/humble/lib:$LD_LIBRARY_PATH"

SCENE_PATH="/home/veni/puppy_ws/src/puppy_worlds/worlds/home_coppelia.ttt"
COPPELIA_DIR="$HOME/CoppeliaSim"

echo "=== 启动 CoppeliaSim headless ==="
cd "$COPPELIA_DIR"
setsid ./coppeliaSim.sh -h "$SCENE_PATH" > /tmp/coppelia.log 2>&1 < /dev/null &
COPPELIA_PID=$!
echo "CoppeliaSim PID: $COPPELIA_PID"

# 等待ZMQ端口就绪
echo "等待ZMQ端口(23000)就绪..."
for i in $(seq 1 30); do
    if ss -tlnp 2>/dev/null | grep -q ':23000'; then
        echo "ZMQ端口已就绪 (等待${i}秒)"
        break
    fi
    sleep 1
    # 检查进程是否还活着
    if ! kill -0 $COPPELIA_PID 2>/dev/null; then
        echo "CoppeliaSim进程已退出！日志:"
        tail -20 /tmp/coppelia.log
        exit 1
    fi
done

if ! ss -tlnp 2>/dev/null | grep -q ':23000'; then
    echo "ZMQ端口30秒内未就绪，日志:"
    tail -30 /tmp/coppelia.log
    exit 1
fi

# 等待仿真自动启动（场景中有自动启动脚本）
sleep 5

# 检查仿真状态
python3 -c "
import sys
sys.path.insert(0, '$COPPELIA_DIR/programming/zmqRemoteApi/clients/python')
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
client = RemoteAPIClient()
sim = client.getObject('sim')
state = sim.getSimulationState()
print(f'仿真状态: {state} (running={sim.simulation_advancing_running})')
if state != sim.simulation_advancing_running:
    print('启动仿真...')
    sim.startSimulation()
    import time
    time.sleep(2)
    state = sim.getSimulationState()
    print(f'仿真状态: {state}')
if state == sim.simulation_advancing_running:
    print('✓ 仿真运行成功！')
else:
    print('✗ 仿真未运行')
    sys.exit(1)
"

echo "=== CoppeliaSim 启动完成 ==="
