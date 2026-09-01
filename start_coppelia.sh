#!/bin/bash
# 启动 CoppeliaSim 并加载场景、附加脚本、启动仿真
source /opt/ros/humble/setup.bash
source ~/puppy_ws/install/setup.bash
export DISPLAY=:1
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

echo "=== 启动 CoppeliaSim ==="
cd ~/CoppeliaSim
./coppeliaSim.sh -h &
COPPELIA_PID=$!
echo "CoppeliaSim PID: $COPPELIA_PID"

echo "等待 CoppeliaSim 启动 (15秒)..."
sleep 15

echo "=== 运行附加脚本 ==="
python3 -u /mnt/e/puppyfangzhen/reattach_script.py
RESULT=$?
echo "附加脚本退出码: $RESULT"

echo "=== 检查 ROS2 话题 ==="
sleep 2
ros2 topic list

echo "=== 完成 ==="
