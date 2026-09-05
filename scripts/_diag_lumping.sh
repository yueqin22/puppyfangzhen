#!/usr/bin/env bash
# Diagnose whether Gazebo lumps the robot into a single link when the leg joints
# are fixed, and report the settled height.
#
# Why: with fixed joints, /link_states lists only `puppy::base_footprint`, and
# base_footprint is an empty link with no <inertial>. If Gazebo lumps every
# welded link into that root, the resulting body may carry no mass, which would
# explain why welding collapsed the commanded velocity to a quarter even with
# friction removed. Comparing the link count between the free-joint and welded
# builds shows whether lumping happens at all.
set +u
export LANG=C
export PATH="$HOME/.local/bin:$PATH"
WS="${WS:-$HOME/puppy_ws}"
WORLD="${WORLD:-small_room}"
SRC="${SRC:-/mnt/e/puppyfangzhen/src}"

for s in /opt/ros/humble/setup.bash "$WS/install/setup.bash"; do
  [[ -f "$s" ]] && source "$s"
done

echo "=== sync + build ==="
rsync -a --delete "$SRC/puppy_description/" "$WS/src/puppy_description/"
cd "$WS" || exit 2
colcon build --packages-select puppy_description 2>&1 | tail -n 2
source "$WS/install/setup.bash"

echo "=== joint types in use ==="
(cd "$WS/src/puppy_description/urdf" && xacro puppy.urdf.xacro use_planar_move:=true 2>/dev/null) \
  | grep -oE '<joint name="FR_(hip_yaw|hip_pitch|knee)_joint" type="[a-z]+"' | head -n 3

echo "=== restart world ==="
pkill -9 -x gzserver; pkill -9 -x gzclient; pkill -9 -x async_slam_toolbox_node
for i in $(seq 1 20); do
  pgrep -x gzserver >/dev/null 2>&1 || break
  sleep 1
done
pgrep -x gzserver >/dev/null 2>&1 && { echo "gzserver alive"; exit 4; }
ros2 daemon stop >/dev/null 2>&1
sleep 2

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
[ "$ready" -eq 1 ] || { echo "world never came up"; tail -n 10 "$WS/world.log"; exit 3; }

sleep 20

echo
echo "=== links reported by Gazebo ground truth ==="
timeout 15 ros2 topic echo --once /link_states --field name 2>/dev/null \
  | tr ',' '\n' | grep puppy | sed "s/^/  /"

echo
echo "=== settled model pose ==="
bash /mnt/e/puppyfangzhen/scripts/_read_pose.sh 2>&1 | tail -n 2

echo
echo "=== axis sweep ==="
python3 /mnt/e/puppyfangzhen/tools/probe_strafe.py --sweep 2>&1 | tail -n 8
echo "DIAG_DONE"
