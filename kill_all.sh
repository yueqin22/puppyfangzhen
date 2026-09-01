#!/bin/bash
# 杀掉所有相关进程
for pattern in coppeliaSim coppelia_bridge "ros2 launch" "ros2 run" nav2 amcl robot_state_publisher rviz controller_server planner_server bt_navigator behavior_server lifecycle_manager smoother puppy_ gazebo; do
    pids=$(pgrep -f "$pattern" 2>/dev/null)
    if [ -n "$pids" ]; then
        kill -9 $pids 2>/dev/null
        echo "已杀掉 $pattern: $pids"
    fi
done
sleep 2

# 清理 SHM
rm -rf /dev/shm/fastrtps* 2>/dev/null
rm -rf /tmp/fastrtps* 2>/dev/null

# 重启 ROS2 daemon
ros2 daemon stop 2>/dev/null
sleep 1
ros2 daemon start 2>/dev/null

# 验证
remaining=$(ps aux | grep -E 'coppelia|ros2|nav2|amcl|rviz|puppy|bridge' | grep -v grep | wc -l)
echo "剩余相关进程数: $remaining"
echo "清理完成"
