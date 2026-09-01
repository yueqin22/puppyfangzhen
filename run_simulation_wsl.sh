#!/usr/bin/env bash
# ==============================================================================
# Puppy Robot + MiniCPM-RobotTrack 3D 物理仿真一键启动脚本 (WSL2 Ubuntu 22.04)
# ==============================================================================

set -e

export PUPPY_SRC=${PUPPY_SRC:-/mnt/e/puppyfangzhen}
export PUPPY_WS=${PUPPY_WS:-$HOME/puppy_ws}
WORLD=${1:-small_room}

echo "============================================================"
echo "  启动 Puppy + MiniCPM-RobotTrack 3D 物理仿真"
echo "  源码路径: $PUPPY_SRC"
echo "  工作空间: $PUPPY_WS"
echo "  仿真场景: $WORLD"
echo "============================================================"

# 1. 检查并同步代码到 WSL 工作空间
echo "[1/4] 同步最新源码到 WSL 工作空间..."
mkdir -p "$PUPPY_WS/src"
cp -r "$PUPPY_SRC/src/"* "$PUPPY_WS/src/"

# 2. 加载 ROS 2 环境并编译
echo "[2/4] 编译 ROS 2 工作空间..."
source /opt/ros/humble/setup.bash
cd "$PUPPY_WS"
colcon build --symlink-install --packages-up-to puppy_minicpm_robot puppy_bringup puppy_description

# 3. 激活工作空间环境
source "$PUPPY_WS/install/setup.bash"

# 4. 启动 3D 物理仿真环境与视觉跟踪节点
echo "[3/4] 启动 Gazebo + RViz2 3D 物理世界与传感器..."
echo "------------------------------------------------------------"
echo "提示:"
echo "  1. Gazebo 窗口和 RViz2 窗口将在 Windows 桌面弹出"
echo "  2. 在另一个终端中可运行:"
echo "     ros2 launch puppy_minicpm_robot minicpm_robot_sim.launch.py mode:=sim"
echo "  3. 发送任务指令测试:"
echo "     ros2 topic pub --once /mission/command std_msgs/msg/String \"{data: '巡视后院并检查是否有可疑物品'}\""
echo "------------------------------------------------------------"

ros2 launch puppy_bringup mapping.launch.py world:="$WORLD" gui:=true
