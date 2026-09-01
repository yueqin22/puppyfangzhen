#!/usr/bin/env bash
# verify_standby_live.sh -- confirm the "no-mission -> standby" fix on the live sim.
#
# What it proves: with NO mission dispatched, the track_cmd_adapter must hold the
# base still (safety_status.standby == true AND /cmd_vel ~ 0). Before this fix the
# tracker auto-followed the mock backend's phantom target (dx=+0.6) and rammed the
# +x wall before any mission was even dispatched, so the mission started against
# the wall.
#
# Usage:
#   ./verify_standby_live.sh            # standby check only (no gazebo needed)
#   ./verify_standby_live.sh --mission  # also dispatch a mission and watch phases
#                                       #   (requires gazebo + gait_controller running)
#
# Environment overrides:
#   SRC   host source package dir  (default /mnt/e/puppyfangzhen/src/puppy_minicpm_robot)
#   WS    colcon workspace          (default $HOME/puppy_ws)
set -uo pipefail
export LANG=C
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

SRC="${SRC:-/mnt/e/puppyfangzhen/src/puppy_minicpm_robot}"
WS="${WS:-$HOME/puppy_ws}"
PKG="$WS/src/puppy_minicpm_robot"
LOG="$WS/verify_standby.log"
WITH_MISSION=0
[[ "${1:-}" == "--mission" ]] && WITH_MISSION=1

: > "$LOG"
exec > "$LOG" 2>&1
set +e

# --- ROS setup: source any installed ROS 2 distro ---
for s in /opt/ros/humble/setup.bash /opt/ros/*/setup.bash; do
  [[ -f "$s" ]] && source "$s"
done
if ! command -v ros2 >/dev/null 2>&1; then
  echo "ERROR: ros2 not on PATH after sourcing ROS setup. Install ROS 2 or override SRC/WS."
  exit 2
fi

cd "$WS" || { echo "ERROR: cannot cd to $WS"; exit 2; }

echo "=== rsync $SRC -> $PKG ==="
rsync -a --delete "$SRC/" "$PKG/"

echo "=== colcon build (puppy_minicpm_robot) ==="
colcon build --symlink-install --packages-select puppy_minicpm_robot 2>&1 | tail -20
source "$WS/install/setup.bash"

echo "=== kill stale nodes ==="
for n in track_cmd_adapter_node mission_grounder_node minicpm_track_node vision_bridge_node; do
  pkill -f "$n" 2>/dev/null
done
sleep 3

echo "=== launch 4-node sim (mode=sim) ==="
setsid --fork ros2 launch puppy_minicpm_robot minicpm_robot_sim.launch.py \
  mode:=sim backend:=mock camera_source:=synthetic > "$WS/launch.log" 2>&1
sleep 14
echo "=== nodes up ==="
ros2 node list

echo "=== STANDBY WINDOW (no mission dispatched yet) ==="
for i in $(seq 1 8); do
  ros2 topic echo /minicpm_robot/safety_status --field data --once 2>/dev/null
  sleep 1
done

echo "=== /cmd_vel linear.x during standby (expect 0.0) ==="
for i in $(seq 1 5); do
  ros2 topic echo /cmd_vel --field linear.x --once 2>/dev/null
  sleep 1
done

STANDBY_HITS=$(grep -c '"standby": true' "$LOG" || true)
CMD_NONZERO=$(grep -E '^[0-9]' "$LOG" | awk '($1+0 != 0){c++} END{print c+0}')
echo "standby_true_samples=$STANDBY_HITS"
echo "nonzero_cmdvel_samples=$CMD_NONZERO"
if [ "$STANDBY_HITS" -ge 1 ] && [ "$CMD_NONZERO" -eq 0 ]; then
  echo "VERDICT: PASS -- base stands by idle before any mission (no wall drive)"
else
  echo "VERDICT: CHECK -- standby=$STANDBY_HITS nonzero_cmd=$CMD_NONZERO"
fi

if [ "$WITH_MISSION" -eq 1 ]; then
  echo "=== dispatch mission: patrol living room ==="
  ros2 topic pub --once /mission/command std_msgs/String \
    "{data: 'Go to the living room and follow the person there, then return home'}"
  echo "=== watch phases for 60s ==="
  for i in $(seq 1 60); do
    ros2 topic echo /minicpm_robot/mission_status --field data --once 2>/dev/null \
      | python3 -c "import sys,json;d=json.loads(sys.stdin.read());print(d.get('phase'))" 2>/dev/null
    sleep 1
  done
fi

echo "=== DONE ==="
for n in track_cmd_adapter_node mission_grounder_node minicpm_track_node vision_bridge_node; do
  pkill -f "$n" 2>/dev/null
done
