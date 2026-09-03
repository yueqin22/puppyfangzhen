#!/usr/bin/env bash
# Sweep the /cmd_vel publish rate and watch the measured efficiency.
#
# Hypothesis: the measured axis response depends on how often /cmd_vel is
# published. If the driver decays the base's velocity between commands, a probe
# publishing slowly measures its own publish rate instead of the robot -- and
# that looks exactly like "the robot is dragging its legs".
#
# Two methodological traps, both of which invalidated earlier attempts:
#
#   * Walking into a wall. A first version drove forward repeatedly; the
#     displacement collapsed 0.106 -> 0.013 -> 0.000 m, which is the robot
#     pinned against the far wall, NOT a rate effect. Fixed by alternating
#     forward and backward runs so the base stays near where it started, and by
#     reporting both directions so a collapse stays visible instead of silent.
#   * Starting from an unknown pose left over by the previous experiment, which
#     may already be against an obstacle. Fixed by restarting the world first.
set +u
export LANG=C
export PATH="$HOME/.local/bin:$PATH"
WS="${WS:-$HOME/puppy_ws}"
WORLD="${WORLD:-small_room}"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

echo "=== hard cleanup ==="
pkill -9 -x gzserver; pkill -9 -x gzclient; pkill -9 -x async_slam_toolbox_node
for i in $(seq 1 20); do
  pgrep -x gzserver >/dev/null 2>&1 || break
  sleep 1
done
if pgrep -x gzserver >/dev/null 2>&1; then
  echo "gzserver will not die"; exit 4
fi
ros2 daemon stop >/dev/null 2>&1
sleep 2

echo "=== fresh world ==="
cd "$WS" || exit 2
setsid --fork ros2 launch puppy_worlds simulation.launch.py \
  world:="$WORLD" gui:=false use_sim_time:=true > "$WS/world.log" 2>&1
setsid --fork ros2 launch puppy_slam slam_toolbox.launch.py > "$WS/slam.log" 2>&1

ready=0
for i in $(seq 1 90); do
  if timeout 8 ros2 topic echo --once /odom --field pose.pose.position.x >/dev/null 2>&1; then
    echo "odom flowing after ${i}s"; ready=1; break
  fi
  sleep 1
done
if [ "$ready" -ne 1 ]; then
  echo "world never came up"; tail -n 10 "$WS/world.log"; exit 3
fi

DUR=1.5
IDEAL=$(python3 -c "print('%.3f' % (0.1 * $DUR))")
echo
echo "forward then backward at vx=+/-0.1 for ${DUR}s (ideal ${IDEAL} m each way)"
echo "rate_hz   fwd_m    back_m   worst_rtf"
for r in 10 200 50 10 200 50; do
  f=$(python3 /mnt/e/puppyfangzhen/tools/probe_strafe.py 0.1 0 0 "$DUR" --rate "$r" 2>&1)
  b=$(python3 /mnt/e/puppyfangzhen/tools/probe_strafe.py -0.1 0 0 "$DUR" --rate "$r" 2>&1)
  fwd=$(echo "$f" | grep "body forward" | grep -oE '[+-][0-9]+\.[0-9]+' | head -n 1)
  bak=$(echo "$b" | grep "body forward" | grep -oE '[+-][0-9]+\.[0-9]+' | head -n 1)
  rtf=$(printf '%s\n%s\n' "$f" "$b" | grep "sim elapsed" \
        | grep -oE 'RTF [0-9.]+' | awk '{print $2}' | sort -n | head -n 1)
  echo "  $r      $fwd    $bak    ${rtf}"
done
echo "RATE_SWEEP_DONE"
