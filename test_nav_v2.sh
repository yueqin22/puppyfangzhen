#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/veni/puppy_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

echo "=== Step 1: Check /goal_pose topic ==="
echo "Publisher count:"
ros2 topic info /goal_pose 2>&1
echo ""
echo "Monitoring /goal_pose for 5 seconds..."
timeout 5 ros2 topic echo /goal_pose 2>&1 | head -20

echo ""
echo "=== Step 2: Kill RViz (eliminate goal source) ==="
pkill -f 'rviz2' 2>/dev/null || true
sleep 2
echo "RViz killed."

echo ""
echo "=== Step 3: Cancel all active Nav2 goals ==="
# Cancel all goals on navigate_to_pose action
ros2 action list 2>&1
echo ""

echo "=== Step 4: Check full /odom orientation ==="
timeout 3 ros2 topic echo /odom --once 2>&1 | grep -A 15 "orientation:" | head -20

echo ""
echo "=== Step 5: Restart only Nav2 (keep Gazebo) ==="
pkill -f 'nav2' 2>/dev/null || true
pkill -f 'ros2 launch.*puppy_nav' 2>/dev/null || true
sleep 3
echo "Nav2 killed."

# Start Nav2 fresh
nohup ros2 launch puppy_nav navigation.launch.py > /home/veni/nav2.log 2>&1 &
echo "Nav2 restarting... waiting 30s"
sleep 30

echo ""
echo "=== Step 6: Check Nav2 state ==="
for node in map_server amcl planner_server controller_server bt_navigator; do
    state=$(ros2 lifecycle get "/${node}" 2>&1)
    echo "  ${node}: ${state}"
done

echo ""
echo "=== Step 7: Check TF after fresh Nav2 ==="
timeout 5 ros2 run tf2_ros tf2_echo map base_footprint 2>&1 | tail -8

echo ""
echo "=== Step 8: Check /odom ==="
timeout 3 ros2 topic echo /odom --once 2>&1 | grep -A 8 "position:" | head -10

echo ""
echo "=== Step 9: Send test goal (1.0, 0.0) ==="
python3 /mnt/e/puppyfangzhen/send_goal.py 1.0 0.0 0.0 2>&1 &

# Monitor /cmd_vel for 10 seconds
echo ""
echo "=== Monitoring /cmd_vel for 10 seconds ==="
timeout 10 ros2 topic echo /cmd_vel 2>&1 | head -30

echo ""
echo "=== Nav2 log (last 20 lines) ==="
tail -20 /home/veni/nav2.log 2>/dev/null
