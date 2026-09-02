#!/usr/bin/env bash
# Test whether a low standby clearance is inherited from the previous run.
#
# Hypothesis: Gazebo keeps the base where the last mission left it when only the
# ROS nodes are restarted, so the standby reading at the start of a verify run is
# really the clearance at the PREVIOUS run's end pose. If true, a "0.27 m at
# standby" is a pose artifact rather than a self-return problem, and the fix
# belongs in the verify procedure (restart the world / assert a known start
# pose), not in the obstacle logic.
#
#   A = clearance right after launch (base where the last run left it)
#   B = clearance after a full mission completes (base at the return pose)
#   C = clearance after restarting ONLY the nodes -> start of the "next" run
#
# If B and C agree, standby clearance is pure carry-over.
#
# Run from WSL:  bash run_pose_carryover_test.sh

WS="${WS:-$HOME/puppy_ws}"
NOODES="[m]inicpm_track_node [t]rack_cmd_adapter_node [v]ision_bridge_node [m]ission_grounder_node"

# No `set -u`: ROS 2 / colcon setup scripts dereference unbound variables.
set -o pipefail
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

kill_nodes() {
  for n in $NOODES; do pkill -f "$n" 2>/dev/null; done
}

wait_for_scan() {
  for i in $(seq 1 60); do
    if timeout 8 ros2 topic echo /scan --field ranges --once > /dev/null 2>&1; then
      echo "    (scan ready after ${i}s)"
      return 0
    fi
    sleep 1
  done
  echo "    (scan NOT seen)"
  return 1
}

kill_nodes
sleep 3
setsid --fork ros2 launch puppy_minicpm_robot minicpm_robot_sim.launch.py \
  mode:=sim backend:=mock camera_source:=synthetic > /home/veni/carryover_launch.log 2>&1
for i in $(seq 1 40); do
  if ros2 node list 2>/dev/null | grep -q track_cmd_adapter; then break; fi
  sleep 1
done
wait_for_scan
echo "=== A: after launch (base inherited from the previous run) ==="
python3 /mnt/e/puppyfangzhen/tools/probe_pose_clearance.py A

echo
echo "=== dispatch mission ==="
for i in $(seq 1 45); do
  n=$(ros2 topic info /mission/command 2>/dev/null | grep 'Subscription count' | grep -oE '[0-9]+')
  n=${n:-0}
  if [ "$n" -ge 1 ]; then break; fi
  sleep 1
done
timeout 15 ros2 topic pub --once /mission/command std_msgs/String \
  "{data: 'Go to living_room and follow the person for 5s, then return home'}" \
  > /home/veni/carryover_pub.txt 2>&1
tail -1 /home/veni/carryover_pub.txt

# Watch for a terminal phase, wall-clock bounded.
deadline=$(( $(date +%s) + 240 ))
final=""
while (( $(date +%s) < deadline )); do
  final=$(timeout 15 ros2 topic echo /minicpm_robot/mission_status --field data --once 2>/dev/null \
    | head -c 4000 | python3 -c "
import json,sys
last=''
for line in sys.stdin:
    line=line.strip()
    if line.startswith('data: '): line=line[6:]
    if not line.startswith('{'): continue
    try: last=json.loads(line).get('phase','')
    except Exception: pass
print(last)")
  case "$final" in
    COMPLETED|FAILED|ABORTED) break ;;
  esac
  sleep 2
done
echo "mission final phase: ${final:-unknown}"

echo
echo "=== B: after the mission (base at wherever it finished) ==="
python3 /mnt/e/puppyfangzhen/tools/probe_pose_clearance.py B

echo
echo "=== restart ONLY the ROS nodes (Gazebo/world untouched) ==="
kill_nodes
sleep 3
setsid --fork ros2 launch puppy_minicpm_robot minicpm_robot_sim.launch.py \
  mode:=sim backend:=mock camera_source:=synthetic > /home/veni/carryover_launch2.log 2>&1
for i in $(seq 1 40); do
  if ros2 node list 2>/dev/null | grep -q track_cmd_adapter; then break; fi
  sleep 1
done
wait_for_scan

echo "=== C: start of the next run (this is what 'standby' would report) ==="
python3 /mnt/e/puppyfangzhen/tools/probe_pose_clearance.py C

echo
echo "=== verdict ==="
echo "If B and C match, the standby clearance is inherited from the previous run's"
echo "end pose -- restart the world (or assert a start pose) before trusting it."

kill_nodes
echo CARRYOVER_DONE
