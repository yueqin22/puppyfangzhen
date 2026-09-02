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
#   ./verify_standby_live.sh                     # standby check only
#   ./verify_standby_live.sh --mission           # also dispatch a mission and watch phases
#   ./verify_standby_live.sh --mission --restart-world
#                                                # restart Gazebo+SLAM first (see below)
#
# --restart-world exists because the sim is a long-lived process, not a fixture.
# Observed: a gzserver left running ~42 h had the base sunk below the floor
# (odom z = -0.155 m) with ~200 deg of the LiDAR arc reading 0.10-0.20 m (floor
# returns). Every navigation then aborted as "blocked by obstacle" no matter what
# the navigation code did -- a phantom failure. Restart the world for a run you
# intend to trust, or use `check_env` to at least detect the rot.
#
# Environment overrides:
#   SRC    host source package dir (default /mnt/e/puppyfangzhen/src/puppy_minicpm_robot)
#   WS     colcon workspace         (default $HOME/puppy_ws)
#   WORLD  Gazebo world name for --restart-world (default small_room)
#   MISSION_TEXT / MISSION_TIMEOUT  mission dispatch overrides
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
CMDYLOG="$WS/verify_standby_cmdvel_y.txt"
INTENTLOG="$WS/verify_standby_intent.txt"
WITH_MISSION=0
RESTART_WORLD=0
for arg in "$@"; do
  case "$arg" in
    --mission) WITH_MISSION=1 ;;
    --restart-world) RESTART_WORLD=1 ;;
  esac
done
WORLD="${WORLD:-small_room}"

NODES=(vision_bridge_node minicpm_track_node track_cmd_adapter_node mission_grounder_node)

# echo_once <topic> [field] [timeout_secs]
# `ros2 topic echo --once` BLOCKS FOREVER when the topic has no publisher -- it does not
# fail fast. Every probe is therefore wrapped in `timeout`, otherwise a stale/absent
# publisher hangs the whole verification instead of reporting it.
echo_once() {
  local topic="$1" field="${2:-}" secs="${3:-8}"
  # stderr is merged on purpose: discarding it hides the only clue when a probe
  # times out (e.g. "topic does not appear to be published yet"), and consumers
  # here only match lines starting with `data: ` / a number, so it is harmless.
  if [[ -n "$field" ]]; then
    timeout "$secs" ros2 topic echo "$topic" --field "$field" --once 2>&1
  else
    timeout "$secs" ros2 topic echo "$topic" --once 2>&1
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

# wait_for_topic <topic> <timeout_secs> -- poll until the topic has a publisher.
wait_for_topic() {
  local topic="$1" deadline=$(( $(date +%s) + ${2:-60} )) n
  while (( $(date +%s) < deadline )); do
    n="$(ros2 topic info "$topic" 2>/dev/null \
         | sed -n 's/.*Publisher count: \([0-9]*\).*/\1/p')"
    if [[ "${n:-0}" =~ ^[0-9]+$ ]] && (( n >= 1 )); then
      echo "$topic: publisher up"
      return 0
    fi
    sleep 2
  done
  echo "WARNING: $topic has no publisher after ${2:-60}s"
  return 1
}

# topic_subs <topic> -- print the subscription count (0 when unavailable).
topic_subs() {
  ros2 topic info "$1" 2>/dev/null \
    | sed -n 's/.*Subscription count: \([0-9]*\).*/\1/p' | head -1
}

# restart_world -- bring Gazebo + SLAM back up from scratch.
# This kills gzserver/slam_toolbox, so it is opt-in (--restart-world).
restart_world() {
  echo "=== restart sim world ($WORLD) ==="
  pkill -9 -f "[g]zserver" 2>/dev/null
  pkill -9 -f "[g]zclient" 2>/dev/null
  pkill -9 -f "[a]sync_slam_toolbox_node" 2>/dev/null
  pkill -9 -f "[r]obot_state_publisher" 2>/dev/null
  pkill -9 -f "[g]ait_controller" 2>/dev/null
  sleep 4
  setsid --fork ros2 launch puppy_worlds simulation.launch.py \
    world:="$WORLD" gui:=false use_sim_time:=true > "$WS/world.log" 2>&1
  setsid --fork ros2 launch puppy_slam slam_toolbox.launch.py > "$WS/slam.log" 2>&1
  wait_for_topic /scan 90
  wait_for_topic /odom 90
  # Gravity/physics need a moment to settle the base onto the floor; judging the
  # pose before that reads the spawn transient, not the steady state.
  echo "settling 15s after world start"
  sleep 15
}

# check_env -- is the robot physically able to navigate at all?
# Prints a verdict on stdout. Sets ENV_OK=1 when the base is on the floor with
# usable free space. Without this, a world that has rotted (base sunk into the
# floor) reports "blocked by obstacle" and the failure gets blamed on navigation.
check_env() {
  local odomf="$WS/verify_env_odom.txt" scanf="$WS/verify_env_scan.txt"
  : > "$odomf"; : > "$scanf"
  # Ask for the position sub-struct, not the whole message: on a full Odometry
  # echo, a bare `z:` regex is ambiguous (position z, orientation z, twist z all
  # match) and silently reports whichever comes first.
  echo_once /odom pose.pose.position 10 > "$odomf"
  echo_once /scan ranges 10 > "$scanf"
  ENV_OK=0
  python3 - "$odomf" "$scanf" <<'PY'
import ast, re, sys
odom_txt = open(sys.argv[1], errors="replace").read()
scan_txt = open(sys.argv[2], errors="replace").read()

z = None
m = re.search(r"z:\s*(-?[0-9.]+)", odom_txt)
if m:
    z = float(m.group(1))
mx = re.search(r"x:\s*(-?[0-9.]+)", odom_txt)
my = re.search(r"y:\s*(-?[0-9.]+)", odom_txt)
pose = "%s, %s" % (("%.3f" % float(mx.group(1))) if mx else "?",
                   ("%.3f" % float(my.group(1))) if my else "?")

ranges = []
m = re.search(r"array\('f',\s*\[(.*?)\]\)", scan_txt, re.S)
if m:
    try:
        ranges = [float(v) for v in m.group(1).split(",") if v.strip()]
    except Exception:
        ranges = []

near = sum(1 for r in ranges if r < 0.35)
frac = (near / len(ranges)) if ranges else 1.0

print("  start pose (odom)  : %s" % pose)
print("  odom z            : %s" % ("?" if z is None else "%.3f" % z))
print("  scan rays         : %d" % len(ranges))
print("  rays closer than 0.35 m: %.0f%%" % (frac * 100))
print("  NOTE: Gazebo keeps the base where the last run left it; only --restart-world")
print("        gives a known start. A low clearance here is usually inherited, not a")
print("        sensor fault -- it was once mistaken for self-returns from the legs.")

# Do NOT treat a slightly negative z as "sunk into the floor": base_footprint on
# this URDF sits ABOUT -0.155 m at rest (measured after a clean world restart,
# with 0% of LiDAR rays closer than 0.35 m and 1.43 m of clearance ahead). Only a
# deep drop means the physics has actually collapsed.
ok = True
if z is not None and z < -0.5:
    print("  UNHEALTHY: base has dropped far below its rest height (z = %.3f)" % z)
    ok = False
if not ranges:
    print("  UNHEALTHY: no LiDAR data")
    ok = False
elif frac > 0.40:
    print("  UNHEALTHY: %.0f%% of rays read < 0.35 m -- the base is hemmed in or grounded"
          % (frac * 100))
    ok = False
elif frac > 0.10:
    # Not unhealthy -- this is a legitimate (if unlucky) start, and the mission
    # may still complete. But it must be visible: a run that begins 0.27 m from a
    # wall exercises obstacle handling, not navigation, and a naive reader would
    # otherwise compare it against a run that started in the open.
    print("  WARN: %.0f%% of rays read < 0.35 m -- the mission starts hemmed in."
          % (frac * 100))
    print("        Pass --restart-world to begin from a known pose.")
if ok:
    print("  env health: OK")
sys.exit(0 if ok else 1)
PY
  [[ $? -eq 0 ]] && ENV_OK=1
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

# Restart the ROS 2 daemon. After several back-to-back runs it goes stale: every
# launched node dies immediately with `ExternalShutdownException` / `RCLError:
# rcl_shutdown already called`, and `ros2 node list` starts failing with an
# xmlrpc parse error -- which reads exactly like a code regression but is not.
# Killing stale nodes is not enough; the daemon has to go too. It restarts
# lazily on the next ros2 call.
echo "=== restart ros2 daemon ==="
ros2 daemon stop >/dev/null 2>&1
sleep 2

# Restart the world BEFORE our nodes launch, so they come up against fresh
# Gazebo/SLAM rather than binding to a sim that is about to be killed.
if [ "$RESTART_WORLD" -eq 1 ]; then
  restart_world
fi

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
: > "$CMDYLOG"
: > "$INTENTLOG"
echo "=== STANDBY WINDOW (no mission dispatched yet) ==="
for i in $(seq 1 8); do
  echo_once /minicpm_robot/safety_status data 8
  echo_once /minicpm_robot/track_intent "" 5 >> "$INTENTLOG"
  echo_once /cmd_vel linear.x 8 >> "$CMDLOG"
  echo_once /cmd_vel linear.y 8 >> "$CMDYLOG"
  sleep 1
done

STANDBY_HITS=$(grep -c '"standby": true' "$LOG" || true)
PHANTOM_HITS=$(grep -c '"target_detected": true' "$INTENTLOG" || true)
CMD_SAMPLES=$(grep -cE '^-?[0-9]' "$CMDLOG" || true)
CMD_NONZERO=$(awk '($1+0 != 0){c++} END{print c+0}' "$CMDLOG")
CMDY_SAMPLES=$(grep -cE '^-?[0-9]' "$CMDYLOG" || true)
CMDY_NONZERO=$(awk '($1+0 != 0){c++} END{print c+0}' "$CMDYLOG")

echo "standby_true_samples=$STANDBY_HITS"
echo "phantom_intent_samples=$PHANTOM_HITS"
echo "cmdvel_samples=$CMD_SAMPLES nonzero_cmdvel_samples=$CMD_NONZERO"
echo "cmdvel_linear_y_samples=$CMDY_SAMPLES nonzero_cmdvel_linear_y=$CMDY_NONZERO"

if [ "$PHANTOM_HITS" -eq 0 ]; then
  echo "VERDICT: INCONCLUSIVE -- no phantom intent seen on /minicpm_robot/track_intent."
  echo "         Standby proves nothing while nothing asks the base to move; this is a"
  echo "         broken setup (tracker not publishing), not a passing fix."
elif [ "$STANDBY_HITS" -ge 1 ] && [ "$CMD_SAMPLES" -ge 1 ] && [ "$CMD_NONZERO" -eq 0 ] \
     && [ "$CMDY_NONZERO" -eq 0 ]; then
  echo "VERDICT: PASS -- base held still under an active phantom target (no wall drive)"
else
  echo "VERDICT: CHECK -- standby=$STANDBY_HITS phantom=$PHANTOM_HITS" \
       "cmd=$CMD_SAMPLES nonzero=$CMD_NONZERO"
fi

if [ "$WITH_MISSION" -eq 1 ]; then
  # The zone keys are UNDERSCORED ("living_room", "backyard"): parse_instruction
  # matches `zone_key in raw_text`, so "living room" with a space silently falls
  # back to default_zone (backyard) and the run quietly verifies a different
  # waypoint than the one you think you asked for.
  MISSION_TEXT="${MISSION_TEXT:-Go to living_room and follow the person for 5s, then return home}"
  MSLOG="$WS/verify_mission_status.txt"
  MCMDLOG="$WS/verify_mission_cmdvel.txt"
  MCMDYLOG="$WS/verify_mission_cmdvel_y.txt"
  SAFELOG="$WS/verify_mission_safety.txt"
  : > "$MSLOG"
  : > "$MCMDLOG"
  : > "$MCMDYLOG"
  : > "$SAFELOG"

  echo "=== environment health (before dispatch) ==="
  check_env

  echo "=== dispatch mission ==="
  echo "instruction: $MISSION_TEXT"
  # Two failure modes to avoid here. (a) `pub --once` firing before discovery:
  # the message goes nowhere and the run looks like a timeout. (b) Publishing
  # repeatedly "just in case": the grounder treats EVERY message as a new mission
  # and resets the phase timer, so `--times 5` created five missions back to back
  # and only the last one ever ran. Wait for the subscriber, then publish once.
  for i in $(seq 1 30); do
    if [[ "$(topic_subs /mission/command)" =~ ^[0-9]+$ ]] \
       && (( $(topic_subs /mission/command) >= 1 )); then
      break
    fi
    sleep 1
  done
  echo "mission/command subscribers: $(topic_subs /mission/command)"
  timeout 15 ros2 topic pub --once /mission/command std_msgs/String \
    "{data: '$MISSION_TEXT'}" 2>&1 | tail -2
  sleep 3

  # Warm up discovery first. The grounder publishes NOTHING until a mission is
  # loaded, so this must run after the dispatch. Under WSL the first
  # publisher<->subscriber match for a fresh topic repeatedly needed far more
  # than the 5 s originally used: every probe timed out, all 42 loop iterations
  # burned their budget waiting, and the run ended INCONCLUSIVE even though the
  # mission had in fact started (cmd_vel went non-zero twice).
  echo "waiting for first mission_status (discovery warm-up, up to 45s)"
  echo_once /minicpm_robot/mission_status data 45 >> "$MSLOG"
  head -c 400 "$MSLOG"

  # Wall-clock bounded, not iteration bounded: the whole cycle is
  # navigate(<=35s) + dwell(5s) + inspect + return(<=35s), so a fixed 60-iteration
  # loop truncates the mission mid-flight and reports a bogus "stuck in phase X".
  MISSION_TIMEOUT="${MISSION_TIMEOUT:-240}"
  echo "=== watch mission (timeout ${MISSION_TIMEOUT}s) ==="
  deadline=$(( $(date +%s) + MISSION_TIMEOUT ))
  final=""
  while (( $(date +%s) < deadline )); do
    echo_once /minicpm_robot/mission_status data 15 >> "$MSLOG"
    echo_once /cmd_vel linear.x 10 >> "$MCMDLOG"
    # lateral command: non-zero here is the signal that the strafe path ran
    echo_once /cmd_vel linear.y 10 >> "$MCMDYLOG"
    # Safety is sampled alongside so a FAILED mission can be attributed: a lidar
    # block at ~0.1 m means the base is grounded (sim rot), a block at just under
    # the 0.35 m threshold with no progress means a real obstruction, and
    # phase_gated/navigating tells you who was supposed to be driving at all.
    echo_once /minicpm_robot/safety_status data 10 >> "$SAFELOG"
    final="$(python3 - "$MSLOG" <<'PY'
import json, sys
# `ros2 topic echo --field data --once` prints the RAW VALUE, not `data: <value>`:
# a String payload comes out as a bare JSON object. Filtering on a "data: " prefix
# therefore discards every sample, the loop never sees a terminal phase and burns
# its whole timeout. Strip the prefix if present, then require a JSON object.
last = ""
try:
    with open(sys.argv[1], errors="replace") as fh:
        for line in fh:
            line = line.strip()
            if line.startswith("data: "):
                line = line[6:]
            if not line.startswith("{"):
                continue
            try:
                last = json.loads(line).get("phase", "")
            except Exception:
                pass
except Exception:
    pass
print(last)
PY
)"
    case "$final" in
      COMPLETED|FAILED|ABORTED) break ;;
    esac
    sleep 1
  done

  echo "=== mission phase trace ==="
  python3 - "$MSLOG" <<'PY'
import json, sys
seq, last = [], None
first_pose = last_pose = None
frame = "?"
msg = ""
try:
    with open(sys.argv[1], errors="replace") as fh:
        for line in fh:
            line = line.strip()
            if line.startswith("data: "):
                line = line[6:]
            if not line.startswith("{"):
                continue
            try:
                d = json.loads(line)
            except Exception:
                continue
            ph = d.get("phase")
            if ph and ph != last:
                seq.append(ph)
                last = ph
            if first_pose is None:
                first_pose = d.get("robot_pose")
            last_pose = d.get("robot_pose")
            frame = d.get("pose_frame", frame)
            msg = d.get("message", "")
except Exception as exc:
    print("  (could not parse mission status: %s)" % exc)
    sys.exit(0)

print("  phase order : %s" % (" -> ".join(seq) if seq else "(none)"))
print("  final phase : %s" % last)
if msg:
    print("  message     : %s" % msg)
print("  pose frame  : %s" % frame)
print("  start pose  : %s" % first_pose)
print("  end pose    : %s" % last_pose)
if first_pose and last_pose:
    dist = ((last_pose[0] - first_pose[0]) ** 2 + (last_pose[1] - first_pose[1]) ** 2) ** 0.5
    print("  net displacement: %.3f m" % dist)
PY

  MCMD_SAMPLES=$(grep -cE '^-?[0-9]' "$MCMDLOG" || true)
  MCMD_NONZERO=$(awk '($1+0 != 0){c++} END{print c+0}' "$MCMDLOG")
  MCMDY_SAMPLES=$(grep -cE '^-?[0-9]' "$MCMDYLOG" || true)
  MCMDY_NONZERO=$(awk '($1+0 != 0){c++} END{print c+0}' "$MCMDYLOG")
  MCMDY_PEAK=$(awk '{v=$1+0; if (v<0) v=-v; if (v>m) m=v} END{printf "%.3f", m+0}' "$MCMDYLOG")
  echo "mission_cmdvel_samples=$MCMD_SAMPLES nonzero=$MCMD_NONZERO"
  echo "mission_cmdvel_linear_y_samples=$MCMDY_SAMPLES nonzero=$MCMDY_NONZERO peak=${MCMDY_PEAK}"

  echo "=== safety during mission ==="
  python3 - "$SAFELOG" <<'PY'
import json, sys
from collections import Counter
reasons, dists, gated, nav, esc = Counter(), [], 0, 0, 0
try:
    with open(sys.argv[1], errors="replace") as fh:
        for line in fh:
            line = line.strip()
            if line.startswith("data: "):
                line = line[6:]
            if not line.startswith("{"):
                continue
            try:
                d = json.loads(line)
            except Exception:
                continue
            reasons[d.get("active_override_reason", "?")] += 1
            dists.append(float(d.get("obstacle_distance", 0.0)))
            gated += 1 if d.get("phase_gated") else 0
            nav += 1 if d.get("navigating") else 0
            esc += 1 if d.get("blocked_escalated") else 0
except Exception as exc:
    print("  (no safety samples: %s)" % exc)
    sys.exit(0)
if not reasons:
    print("  (no safety samples collected)")
    sys.exit(0)
print("  samples: %d" % sum(reasons.values()))
for r, c in reasons.most_common():
    print("    %-55s x%d" % (r, c))
dists.sort()
if dists:
    print("  obstacle_distance min=%.3f median=%.3f max=%.3f"
          % (dists[0], dists[len(dists) // 2], dists[-1]))
print("  phase_gated=%d navigating=%d blocked_escalated=%d" % (gated, nav, esc))
PY

  if [ "${ENV_OK:-0}" -ne 1 ]; then
    echo "NOTE: the world was flagged UNHEALTHY before dispatch. A navigation"
    echo "      failure here is far more likely the sim than the navigation code;"
    echo "      re-run with --restart-world before believing it."
  fi

  # A mission that "completes" without the base ever moving is as meaningless as
  # a standby check with no phantom target: the timers would have fired while the
  # robot stood still, so require evidence of motion before believing COMPLETED.
  case "$final" in
    COMPLETED)
      if [ "$MCMD_NONZERO" -eq 0 ]; then
        echo "MISSION VERDICT: FAIL -- reached COMPLETED but /cmd_vel was never non-zero."
        echo "                 The phases advanced on timers alone; the base never drove."
      else
        echo "MISSION VERDICT: PASS -- reached COMPLETED with $MCMD_NONZERO moving samples"
      fi
      ;;
    FAILED|ABORTED)
      echo "MISSION VERDICT: FAIL -- mission ended in $final"
      ;;
    "")
      echo "MISSION VERDICT: INCONCLUSIVE -- no mission_status received; did /mission/command land?"
      ;;
    *)
      echo "MISSION VERDICT: CHECK -- still in phase $final after ${MISSION_TIMEOUT}s"
      ;;
  esac
fi

echo "=== DONE ==="
kill_nodes
