#!/usr/bin/env bash
# Verify that the two URDF modes generate what we expect.
#
# Written as a file rather than a one-liner because shell variables inside
# `wsl bash -c '...'` get eaten by the outer shell even inside single quotes,
# which silently turns a two-case loop into the same default case twice.
set +u
export LANG=C
export PATH="$HOME/.local/bin:$PATH"
WS="${WS:-$HOME/puppy_ws}"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

rsync -a /mnt/e/puppyfangzhen/src/puppy_description/ "$WS/src/puppy_description/"
cd "$WS/src/puppy_description/urdf" || exit 2

for mode in true false; do
  echo "=== use_planar_move:=$mode ==="
  if ! xacro puppy.urdf.xacro use_planar_move:=$mode > "/tmp/pm_$mode.urdf" 2>"/tmp/err_$mode.txt"; then
    echo "XACRO FAILED"
    head -n 6 "/tmp/err_$mode.txt"
    continue
  fi
  grep -oE '<joint name="FR_(hip_yaw|hip_pitch|knee)_joint" type="[a-z]+"' "/tmp/pm_$mode.urdf"
  echo -n "  FR_foot_link mu1/mu2: "
  grep -A8 'reference="FR_foot_link"' "/tmp/pm_$mode.urdf" \
    | grep -oE '<mu[12]>[0-9.]+' | tr '\n' ' '
  echo
  echo -n "  planar plugin present: "
  grep -c 'libgazebo_ros_planar_move.so' "/tmp/pm_$mode.urdf"
  echo -n "  ros2_control present : "
  grep -c 'ros2_control' "/tmp/pm_$mode.urdf"
done
echo "URDF_MODE_CHECK_DONE"
