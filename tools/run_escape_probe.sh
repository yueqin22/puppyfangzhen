#!/usr/bin/env bash
# Run probe_escape.py with the tangential escape disabled and then enabled.
#
# Unlike run_escape_ab.sh this does NOT restart the world: it reuses wherever the
# previous run left the base, which is exactly the pose that produced the 0.33 m
# graze. Restarting Gazebo would put the base back at the spawn point in the
# middle of the room, where nothing is within 2 m and no block can be triggered.
#
# Usage:  bash run_escape_probe.sh [duration_per_arm]
# Output: $WS/escape_probe_{off,on}.json
set -o pipefail
export LANG=C
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export PATH="$HOME/.local/bin:$PATH"

YAML=/mnt/e/puppyfangzhen/src/puppy_minicpm_robot/config/minicpm_robot.yaml
PROBE=/mnt/e/puppyfangzhen/tools/probe_escape.py
WS="${WS:-$HOME/puppy_ws}"
DUR="${1:-40}"

for s in /opt/ros/humble/setup.bash "$WS/install/setup.bash"; do
  [[ -f "$s" ]] && source "$s"
done

cp "$YAML" "$WS/escape_yaml_backup.yaml"
restore() { cp "$WS/escape_yaml_backup.yaml" "$YAML"; echo "=== yaml restored ==="; }
trap restore EXIT

wait_for_nodes() {
  local deadline=$(( $(date +%s) + 60 )) n
  while (( $(date +%s) < deadline )); do
    local listed missing=""
    listed="$(ros2 node list 2>/dev/null)"
    for n in vision_bridge_node minicpm_track_node track_cmd_adapter_node mission_grounder_node; do
      grep -q "^/${n}\$" <<< "$listed" || missing="$missing $n"
    done
    [[ -z "$missing" ]] && { echo "  nodes up"; return 0; }
    sleep 2
  done
  echo "  WARNING: nodes missing after 60s:$missing"
  return 1
}

run_arm() {
  local label="$1" factor="$2"
  echo "=== ARM '$label' (escape_clearance_factor=$factor) ==="
  sed -i "s/^    escape_clearance_factor: .*/    escape_clearance_factor: $factor/" "$YAML"
  rsync -a --delete /mnt/e/puppyfangzhen/src/puppy_minicpm_robot/ "$WS/src/puppy_minicpm_robot/"
  ( cd "$WS" && colcon build --symlink-install --packages-select puppy_minicpm_robot > "$WS/escape_build_${label}.log" 2>&1 )
  source "$WS/install/setup.bash"

  for n in vision_bridge_node minicpm_track_node track_cmd_adapter_node mission_grounder_node; do
    pkill -f "[${n:0:1}]${n:1}" 2>/dev/null
  done
  sleep 3
  ros2 daemon stop >/dev/null 2>&1
  sleep 2

  setsid --fork ros2 launch puppy_minicpm_robot minicpm_robot_sim.launch.py \
    mode:=sim backend:=mock camera_source:=synthetic > "$WS/escape_launch_${label}.log" 2>&1
  wait_for_nodes

  python3 "$PROBE" --duration "$DUR" --out "$WS/escape_probe_${label}.json" \
    2>&1 | tee "$WS/escape_probe_${label}.log"
  sleep 2
}

run_arm off 999.0
run_arm on 1.15

echo "=== comparison ==="
python3 /mnt/e/puppyfangzhen/tools/escape_ab_report.py \
  "$WS/escape_probe_off.json" "$WS/escape_probe_on.json" 2>/dev/null || true
