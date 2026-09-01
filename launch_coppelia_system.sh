#!/bin/bash
# 启动 CoppeliaSim 版完整系统
export PATH="/home/veni/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
source /opt/ros/humble/setup.bash
source ~/puppy_ws/install/setup.bash
export DISPLAY=:1
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
# 禁用 SHM 传输，解决 fastrtps_port7427 锁文件错误导致 /scan 等话题无法接收
export FASTRTPS_DEFAULT_PROFILES_FILE=/mnt/e/puppyfangzhen/fastdds_profile.xml

echo "=== 启动 CoppeliaSim 系统 ==="
ros2 launch puppy_bringup coppelia_system.launch.py
