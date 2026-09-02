#!/usr/bin/env bash
# Prove that the strafe actually reaches /cmd_vel, at the full 20 Hz rate.
#
# The mission verifier only manages a handful of sparse `ros2 topic echo --once`
# samples per run, so it can easily miss every lateral frame and report
# linear.y == 0 even while the mission completes. This script subscribes for the
# whole mission instead, so "was lateral published" is answered by data rather
# than inferred from the outcome.
#
# Run from WSL:  bash probe_lateral_live.sh

WS="${WS:-$HOME/puppy_ws}"
DUR="${DUR:-170}"
STATS="/home/veni/cmdvel_stats.txt"
LAUNCH_LOG="/home/veni/probe_lateral_launch.log"

# NOTE: no `set -u` here. ROS 2 and colcon setup scripts dereference unbound
# variables (AMENT_TRACE_SETUP_FILES, COLCON_TRACE) and abort the script at the
# source line before anything useful runs.
set -o pipefail

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

pkill -f "[m]inicpm_track_node" 2>/dev/null
pkill -f "[t]rack_cmd_adapter_node" 2>/dev/null
pkill -f "[v]ision_bridge_node" 2>/dev/null
pkill -f "[m]ission_grounder_node" 2>/dev/null
sleep 3

setsid --fork ros2 launch puppy_minicpm_robot minicpm_robot_sim.launch.py \
  mode:=sim backend:=mock camera_source:=synthetic > "$LAUNCH_LOG" 2>&1
sleep 30
echo "nodes:"
ros2 node list 2>/dev/null | grep -E "track_cmd_adapter|minicpm_track|mission_grounder"

# Start the recorder BEFORE the dispatch, otherwise the first seconds of
# navigation -- exactly the part we care about -- are missed.
setsid --fork python3 /mnt/e/puppyfangzhen/tools/probe_cmdvel_lateral.py \
  "$DUR" /home/veni/cmdvel_stats.json > "$STATS" 2>&1
sleep 5

echo "waiting for /mission/command subscriber"
for i in $(seq 1 45); do
  n=$(ros2 topic info /mission/command 2>/dev/null | grep 'Subscription count' | grep -oE '[0-9]+')
  n=${n:-0}
  if [ "$n" -ge 1 ]; then break; fi
  sleep 1
done
echo "mission/command subscribers: ${n:-0}"

# Quoting matters: the payload is a YAML mapping for `ros2 topic pub`, so the
# sentence must stay quoted. Dropping the quotes makes ros2 parse "Go" as the
# whole value and complain about the rest as unknown fields. The result is
# teed to a file as well: discovery under WSL reports 0 subscribers often
# enough that "did the publish even happen" has to be checked, not assumed.
timeout 15 ros2 topic pub --once /mission/command std_msgs/String \
  "{data: 'Go to living_room and follow the person for 5s, then return home'}" \
  > /home/veni/probe_pub.txt 2>&1
echo "--- publish result ---"
cat /home/veni/probe_pub.txt

sleep $((DUR + 5))

echo "=== /cmd_vel statistics over the whole mission ==="
cat "$STATS"

echo "=== final mission status ==="
timeout 20 ros2 topic echo /minicpm_robot/mission_status --field data --once 2>&1 | head -c 400
echo

pkill -f "[m]inicpm_track_node" 2>/dev/null
pkill -f "[t]rack_cmd_adapter_node" 2>/dev/null
pkill -f "[v]ision_bridge_node" 2>/dev/null
pkill -f "[m]ission_grounder_node" 2>/dev/null
echo PROBE_DONE
