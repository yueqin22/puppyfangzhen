#!/usr/bin/env bash
# Launch the sim and characterise the near LiDAR returns while the base is at rest.
#
# Kept as a script file rather than an inline `wsl bash -lc '...'` command because
# nested quoting on the Windows->wsl->bash path silently eats quotes, which once
# made a mission dispatch fail while looking like a probe result.
#
# Run from WSL:  bash run_selfreturn_diag.sh

WS="${WS:-$HOME/puppy_ws}"
LOG="/home/veni/selfreturn_diag.log"

# No `set -u`: ROS 2 / colcon setup scripts dereference unbound variables and
# abort the script at the source line before anything runs.
set -o pipefail

source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"

pkill -f "[m]inicpm_track_node" 2>/dev/null
pkill -f "[t]rack_cmd_adapter_node" 2>/dev/null
pkill -f "[v]ision_bridge_node" 2>/dev/null
pkill -f "[m]ission_grounder_node" 2>/dev/null
sleep 3

setsid --fork ros2 launch puppy_minicpm_robot minicpm_robot_sim.launch.py \
  mode:=sim backend:=mock camera_source:=synthetic > /home/veni/selfreturn_launch.log 2>&1

# Poll for the scan rather than sleeping a fixed time: DDS discovery under WSL
# is slow enough that a fixed 30 s regularly misses a perfectly healthy node.
for i in $(seq 1 60); do
  if timeout 8 ros2 topic echo /scan --field ranges --once > /dev/null 2>&1; then
    echo "scan available after ${i}s"
    break
  fi
  sleep 1
done

echo "=== self-return characterisation (base at rest, no mission) ==="
python3 /mnt/e/puppyfangzhen/tools/diagnose_selfreturns.py 2>&1 | tee "$LOG"

pkill -f "[m]inicpm_track_node" 2>/dev/null
pkill -f "[t]rack_cmd_adapter_node" 2>/dev/null
pkill -f "[v]ision_bridge_node" 2>/dev/null
pkill -f "[m]ission_grounder_node" 2>/dev/null
echo DIAG_DONE
