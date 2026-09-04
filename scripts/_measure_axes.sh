#!/usr/bin/env bash
# Measure the base's real /cmd_vel axis response on a freshly spawned world.
#
# Why a fresh world every time: displacement over a fixed command is only
# comparable if the robot starts from the same, unobstructed pose. A robot left
# standing against a wall from the previous run reports a collapsed efficiency
# that looks exactly like a broken actuator.
#
# SOURCE SYNC IS NOT OPTIONAL
# ---------------------------
# The Windows tree (/mnt/e/puppyfangzhen) is authoritative; WSL holds a copy
# that colcon builds. An earlier version of this script built WITHOUT syncing,
# so after a `git checkout` reverted a URDF change on the Windows side, WSL kept
# building and measuring the reverted-away version. Every number produced
# afterwards described a model that no longer existed in the repository. The
# sync below, plus the post-sync verification, is what stops that recurring.
#
# Hard-won cleanup notes:
#   * pkill -f <name> also matches the invoking shell when the name appears in
#     its own command line. Use -x (exact process name), or the [f]irst-character
#     trick with -f, and never place the bare literal in the command running the
#     pkill.
#   * `ros2 topic list` reports /odom even when nothing publishes (stale daemon).
#     Wait for an actual message before trusting the world is up.
#   * colcon lives in ~/.local/bin, which is NOT on a non-login shell's PATH.
#     Without exporting it the build silently no-ops, the install space keeps the
#     previous URDF, and the change appears to have no effect.
#   * pgrep right after pkill reports a process that is already dying (SIGKILL is
#     not instantaneous and zombies linger until reaped). Poll instead of a
#     single sleep, or a healthy run gets aborted for nothing.
set +u
export LANG=C
export PATH="$HOME/.local/bin:$PATH"
WS="${WS:-$HOME/puppy_ws}"
WORLD="${WORLD:-small_room}"
SRC="${SRC:-/mnt/e/puppyfangzhen/src}"

for s in /opt/ros/humble/setup.bash "$WS/install/setup.bash"; do
  [[ -f "$s" ]] && source "$s"
done

# Both the model and the launch that spawns it have to come across: a change to
# the spawn height lives in puppy_worlds, and building without syncing it would
# silently measure the old height.
PKGS="puppy_description puppy_worlds"
echo "=== sync source (Windows tree is authoritative) ==="
for p in $PKGS; do
  rsync -a --delete "$SRC/$p/" "$WS/src/$p/"
done
for p in $PKGS; do
  if ! diff -r "$SRC/$p/" "$WS/src/$p/" >/dev/null 2>&1; then
    echo "!! $p still differs after rsync -- refusing to build"
    diff -rq "$SRC/$p/" "$WS/src/$p/" | head
    exit 5
  fi
done
echo "in sync"

echo "=== build $PKGS ==="
cd "$WS" || exit 2
# shellcheck disable=SC2086
colcon build --packages-select $PKGS 2>&1 | tail -n 3
source "$WS/install/setup.bash"

# Lint before anything expensive. `--` is illegal inside an XML comment, which
# is easy to write by accident in prose and cost a full 7-minute world restart
# to discover. Catch it here, in milliseconds.
#
# A per-line grep misses a `--` that sits on a DIFFERENT line from the `<!--`
# opener, and a span-aware grep|sed pipeline produced false positives on clean
# files. So parse each comment body unambiguously with Python's regex and flag
# any body that itself contains `--` (the one token xacro refuses to parse).
bad=0
shopt -s nullglob
for f in "$WS/src/puppy_description/urdf"/*.xacro "$WS/src/puppy_worlds/worlds"/*.world; do
  if python3 - "$f" <<'PY'
import sys, re
t = open(sys.argv[1], encoding='utf-8', errors='replace').read()
sys.exit(1 if any('--' in m.group(1) for m in re.finditer(r'<!--(.*?)-->', t, re.S)) else 0)
PY
  then
    :
  else
    echo "!! $f: double hyphen inside an XML comment -- xacro will refuse to parse"
    bad=1
  fi
done
if [ "$bad" -ne 0 ]; then
  exit 6
fi

# Validate the URDF BEFORE tearing down the world, so a broken model does not
# cost a full restart cycle.
echo "=== joint types the world will actually use ==="
URDF=$(cd "$WS/src/puppy_description/urdf" && xacro puppy.urdf.xacro use_planar_move:=true 2>&1)
if [ $? -ne 0 ] || ! echo "$URDF" | grep -q "<robot"; then
  echo "!! xacro failed -- refusing to restart the world"
  echo "$URDF" | head -n 12
  exit 7
fi
echo "$URDF" | grep -oE '<joint name="FR_[a-z_]+" type="[a-z]+"' | head -n 4

echo "=== hard cleanup ==="
pkill -9 -x gzserver; pkill -9 -x gzclient; pkill -9 -x async_slam_toolbox_node
for i in $(seq 1 20); do
  pgrep -x gzserver >/dev/null 2>&1 || break
  sleep 1
done
if pgrep -x gzserver >/dev/null 2>&1; then
  echo "!! gzserver survived the kill -- refusing to measure a stale world"
  ps -eo pid,ppid,stat,etimes,comm | grep -i gzserver
  exit 4
fi
echo "gzserver down"

echo "=== restart ros2 daemon ==="
ros2 daemon stop >/dev/null 2>&1
sleep 2

echo "=== restart world + slam ==="
setsid --fork ros2 launch puppy_worlds simulation.launch.py \
  world:="$WORLD" gui:=false use_sim_time:=true > "$WS/world.log" 2>&1
setsid --fork ros2 launch puppy_slam slam_toolbox.launch.py > "$WS/slam.log" 2>&1

echo "=== waiting for an actual /odom message ==="
ready=0
for i in $(seq 1 90); do
  if timeout 8 ros2 topic echo --once /odom --field pose.pose.position.x >/dev/null 2>&1; then
    echo "odom flowing after ${i}s"
    ready=1
    break
  fi
  sleep 1
done
if [ "$ready" -ne 1 ]; then
  echo "WORLD NEVER CAME UP -- tail of world.log:"
  tail -n 12 "$WS/world.log"
  exit 3
fi

echo
echo "=== axis sweep (timed against the simulated clock) ==="
python3 /mnt/e/puppyfangzhen/tools/probe_strafe.py --sweep
echo "MEASURE_DONE"
