#!/usr/bin/env bash
# verify_standby_live.sh -- confirm the "no-mission -> standby" fix on the live sim.
#
# What it proves: with NO mission dispatched, track_cmd_adapter must hold the base
# still (safety_status.standby == true AND /cmd_vel ~ 0) *even while* the mock backend
# keeps publishing a phantom tracked target (dx=+0.6). Before the fix the tracker
# auto-followed that phantom and rammed the +x wall before any mission was dispatched,
# so missions started against the wall.
#
# The phantom-intent check is essential. If /minicpm_robot/track_intent carries no
# target_detected intent, "standby" proves nothing -- the base would be still simply
# because nothing asked it to move. That case reports INCONCLUSIVE, never PASS.
#
# Topic names: the intent topic is NAMESPACED -- /minicpm_robot/track_intent, published
# by minicpm_track_node.py and subscribed by track_cmd_adapter_node.py and
# mission_grounder_node.py. Probing a bare /track_intent yields a misleading
# "does not appear to be published yet" and will make you chase a bug that isn't there.
#
# Usage:
#   ./verify_standby_live.sh            # standby check only
#   ./verify_standby_live.sh --mission  # also dispatch a mission and watch phases
#
# Environment overrides:
#   SRC  host source package dir (default /mnt/e/puppyfangzhen/src/puppy_minicpm_robot)
#   WS   colcon workspace         (default $HOME/puppy_ws)
#
# NOTE: deliberately NOT using `set -u`. ROS 2 (/opt/ros/*/setup.bash dereferences
# AMENT_TRACE_SETUP_FILES) and colcon ($WS/install/setup.bash dereferences COLCON_TRACE)
# setup scripts rely on unbound variables; under -u the shell aborts *while sourcing
# them*, so rsync/build never run and the log shows only one cryptic line.
set -o pipefail
export LANG=C
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

SRC="${SRC:-/mnt/e/puppyfangzhen/src/puppy_minicpm_robot}"
WS="${WS:-$HOME/puppy_ws}"
PKG="$WS/src/puppy_minicpm_robot"
LOG="$WS/verify_standby.log"
CMDLOG="$WS/verify_standby_cmdvel.txt"
INTENTLOG="$WS/verify_standby_intent.txt"
WITH_MISSION=0
[[ "${1:-}" == "--mission" ]] && WITH_MISSION=1

NODES=(vision_bridge_node minicpm_track_node track_cmd_adapter_node mission_grounder_node)

# echo_once <topic> [field] [timeout_secs]
# `ros2 topic echo --once` BLOCKS FOREVER when the topic has no publisher -- it does not
# fail fast. Every probe is therefore wrapped in `timeout`, otherwise a stale/absent
# publisher hangs the whole verification instead of reporting it.
echo_once() {
  local topic="$1" field="${2:-}" secs="${3:-8}"
  if [[ -n "$field" ]]; then
    timeout "$secs" ros2 topic echo "$topic" --field "$field" --once 2>/dev/null
  else
    timeout "$secs" ros2 topic echo "$topic" --once 2>/dev/null
  fi
}

# wait_for_nodes <timeout_secs>
# DDS discovery under WSL is slow: Fast-DDS frequently logs
#   [RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_portNNNN
# and falls back to UDP. A fixed sleep is therefore unreliable -- minicpm_track_node was
# legitimately missing from `ros2 node list` at 14s while perfectly healthy. Poll instead.
wait_for_nodes() {
  local deadline=$(( $(date +%s) + ${1:-60} )) listed missing n
  while (( $(date +%s) < deadline )); do
    listed="$(ros2 node list 2>/dev/null)"
    missing=""
    for n in "${NODES[@]}"; do
      grep -q "^/${n}\$" <<< "$listed" || missing="$missing $n"
    done
    if [[ -z "$missing" ]]; then
      echo "all ${#NODES[@]} nodes discovered"
      return 0
    fi
    sleep 2
  done
  echo "WARNING: nodes still missing after ${1:-60}s:$missing"
  return 1
}

# kill_nodes -- the `[x]` bracket trick stops `pkill -f` from matching this script's own
# command line, which would SIGTERM the shell itself (exit 143) mid-verification.
kill_nodes() {
  local n
  for n in "${NODES[@]}"; do
    pkill -f "[${n:0:1}]${n:1}" 2>/dev/null
  done
}

: > "$LOG"
exec > "$LOG" 2>&1
set +e

# --- ROS setup: source any installed ROS 2 distro ---
for s in /opt/ros/humble/setup.bash /opt/ros/*/setup.bash; do
  [[ -f "$s" ]] && source "$s"
done
# `command -v ros2` is unreliable here: ros2 may be exposed as a shell function /
# wrapper that command -v does not report even though ros2 runs fine. Probe by actually
# executing it instead of looking for it on PATH.
if ! ros2 pkg list >/dev/null 2>&1; then
  echo "ERROR: ros2 not usable after sourcing ROS setup. Install ROS 2 or override SRC/WS."
  exit 2
fi

cd "$WS" || { echo "ERROR: cannot cd to $WS"; exit 2; }

echo "=== rsync $SRC -> $PKG ==="
rsync -a --delete "$SRC/" "$PKG/"

echo "=== colcon build (puppy_minicpm_robot) ==="
colcon build --symlink-install --packages-select puppy_minicpm_robot 2>&1 | tail -20
source "$WS/install/setup.bash"

echo "=== kill stale nodes ==="
kill_nodes
sleep 3

echo "=== launch 4-node sim (mode=sim) ==="
setsid --fork ros2 launch puppy_minicpm_robot minicpm_robot_sim.launch.py \
  mode:=sim backend:=mock camera_source:=synthetic > "$WS/launch.log" 2>&1
wait_for_nodes 60
echo "=== nodes up ==="
ros2 node list

echo "=== publisher counts (diagnostics: proves the stress source exists) ==="
ros2 topic info /minicpm_robot/track_intent 2>&1 | grep -E 'Type|count'
ros2 topic info /cmd_vel 2>&1 | grep -E 'Type|count'

# --- STANDBY WINDOW: no mission dispatched yet ---
# Sample safety status, the tracker's phantom intent, and /cmd_vel together, so that
# "standby" can only pass while an active target is actually asking the base to move.
: > "$CMDLOG"
: > "$INTENTLOG"
echo "=== STANDBY WINDOW (no mission dispatched yet) ==="
for i in $(seq 1 8); do
  echo_once /minicpm_robot/safety_status data 8
  echo_once /minicpm_robot/track_intent "" 5 >> "$INTENTLOG"
  echo_once /cmd_vel linear.x 8 >> "$CMDLOG"
  sleep 1
done

STANDBY_HITS=$(grep -c '"standby": true' "$LOG" || true)
PHANTOM_HITS=$(grep -c '"target_detected": true' "$INTENTLOG" || true)
CMD_SAMPLES=$(grep -cE '^-?[0-9]' "$CMDLOG" || true)
CMD_NONZERO=$(awk '($1+0 != 0){c++} END{print c+0}' "$CMDLOG")

echo "standby_true_samples=$STANDBY_HITS"
echo "phantom_intent_samples=$PHANTOM_HITS"
echo "cmdvel_samples=$CMD_SAMPLES nonzero_cmdvel_samples=$CMD_NONZERO"

if [ "$PHANTOM_HITS" -eq 0 ]; then
  echo "VERDICT: INCONCLUSIVE -- no phantom intent seen on /minicpm_robot/track_intent."
  echo "         Standby proves nothing while nothing asks the base to move; this is a"
  echo "         broken setup (tracker not publishing), not a passing fix."
elif [ "$STANDBY_HITS" -ge 1 ] && [ "$CMD_SAMPLES" -ge 1 ] && [ "$CMD_NONZERO" -eq 0 ]; then
  echo "VERDICT: PASS -- base held still under an active phantom target (no wall drive)"
else
  echo "VERDICT: CHECK -- standby=$STANDBY_HITS phantom=$PHANTOM_HITS" \
       "cmd=$CMD_SAMPLES nonzero=$CMD_NONZERO"
fi

if [ "$WITH_MISSION" -eq 1 ]; then
  echo "=== dispatch mission: patrol living room ==="
  timeout 10 ros2 topic pub --once /mission/command std_msgs/String \
    "{data: 'Go to the living room and follow the person there, then return home'}"
  echo "=== watch phases for 60s ==="
  for i in $(seq 1 60); do
    echo_once /minicpm_robot/mission_status data 5 | python3 -c "
import sys, json
try:
    print(json.loads(sys.stdin.read()).get('phase'))
except Exception:
    pass" 2>/dev/null
    sleep 1
  done
fi

echo "=== DONE ==="
kill_nodes
