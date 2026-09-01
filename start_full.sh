#!/bin/bash
# 手动分步启动完整系统
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/humble/setup.bash
source ~/puppy_ws/install/setup.bash
cd ~/puppy_ws

echo "===== Step 1: 启动仿真 ====="
ros2 launch puppy_worlds simulation.launch.py gui:=false &
SIM_PID=$!
echo "仿真 PID: $SIM_PID"

echo "等待 20 秒让仿真完全加载..."
sleep 20

echo "===== Step 2: 启动 Nav2 ====="
ros2 launch puppy_nav navigation.launch.py &
NAV_PID=$!
echo "Nav2 PID: $NAV_PID"

echo "等待 15 秒让 Nav2 完全激活..."
sleep 15

echo "===== Step 3: 启动功能节点 ====="
ros2 run puppy_gait fall_detection.py --ros-args -p use_sim_time:=true &
ros2 run puppy_gait security_node.py --ros-args -p use_sim_time:=true &
ros2 run puppy_gait emotion_interaction.py --ros-args -p use_sim_time:=true &
ros2 run puppy_gait battery_simulator.py --ros-args -p use_sim_time:=true &

echo "等待 5 秒..."
sleep 5

echo "===== Step 4: 启动巡逻 ====="
ros2 run puppy_nav patrol.py --ros-args -p use_sim_time:=true &
PATROL_PID=$!
echo "巡逻 PID: $PATROL_PID"

echo "===== 系统全部启动 ====="
echo "等待巡逻完成..."
wait $PATROL_PID
echo "巡逻结束"
