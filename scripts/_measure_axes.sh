#!/usr/bin/env bash
# Measure the base's real /cmd_vel axis response on a freshly spawned world.
#
# Why a fresh world every time: displacement over a fixed command is only
# comparable if the robot starts from the same, unobstructed pose. A robot left
# standing against a wall from the previous run reports a collapsed efficiency
# that looks exactly like a broken actuator.
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
#   * An old gzserver holding the port makes the new one exit 255, so the whole
#     run silently measures a dead world. Kill first, then verify.
set +u
export LANG=C
WS="${WS:-$HOME/puppy_ws}"
WORLD="${WORLD:-small_room}"
export PATH="$HOME/.local/bin:$PATH"

for s in /opt/ros/humble/setup.bash "$WS/install/setup.bash"; do
  [[ -f "$s" ]] && source "$s"
done
cd "$WS" || exit 2

echo "=== build puppy_description (pick up any URDF change) ==="
colcon build --packages-select puppy_description 2>&1 | tail -n 3
source "$WS/install/setup.bash"

echo "=== hard cleanup ==="
pkill -9 -x gzserver
pkill -9 -x gzclient
pkill -9 -x async_slam_toolbox_node
pkill -9 -f "[s]pawn_entity"
# SIGKILL is not instantaneous: the process lingers for a moment (and shows up
# as a zombie until it is reaped). A single fixed sleep reports a false
# "survived the kill" and aborts a perfectly good run, so poll instead.
gone=0
for i in $(seq 1 20); do
  if ! pgrep -x gzserver >/dev/null 2>&1; then
    echo "gzserver down after ${i}s"
    gone=1
    break
  fi
  sleep 1
done
if [ "$gone" -ne 1 ]; then
  echo "!! gzserver survived the kill -- aborting rather than measuring a stale world"
  ps -eo pid,ppid,stat,etimes,comm | grep -i gzserver
  exit 4
fi

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

echo "=== confirm which URDF the running world uses ==="
timeout 10 ros2 topic echo --once /robot_description 2>/dev/null \
  | grep -oE '<joint name="FR_(hip_yaw|hip_pitch|knee)_joint" type="[a-z]+"' | head -n 3

echo
echo "=== axis sweep (timed against the simulated clock) ==="
python3 /mnt/e/puppyfangzhen/tools/probe_strafe.py --sweep
echo "MEASURE_DONE"
