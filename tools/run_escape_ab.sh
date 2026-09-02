#!/usr/bin/env bash
# A/B the tangential escape (defect 13) on the live sim: same mission, escape off
# then on, recording safety status at full rate both times.
#
# Why an A/B at all: after the fix, two clean end-to-end runs completed with
# blocked_escalated=0 and a minimum clearance of 0.451/0.632 m, up from 0.330/
# 0.339 m before. That is suggestive but not proof -- `verify_standby_live.sh`
# only collects 4-5 safety samples per mission, so "the base never got close to
# a wall" and "the base got close but we missed it" look the same in that log.
# Both arms here are measured with the full-rate recorder, so the comparison is
# like for like.
#
# Measured caveat: this experiment is at the mercy of SLAM drift. The graze that
# motivated the fix only happens when drift happens to route the base near the
# home-point obstacle, and one arm here finished with a 0.566 m minimum clearance
# and zero blocked samples -- it never got close enough to exercise the code
# under test at all. So a null result here proves nothing either way. When you
# need a guaranteed block, use run_escape_probe.sh instead: it aims the goal down
# the narrowest measured bearing, so the base is vetoed by its own LiDAR within
# a second regardless of where it happens to be standing.
#
# Disabling the escape needs no code change: escape_clearance_factor scales the
# clearance a tangent must beat, so 999.0 makes every tangent unacceptable and
# the adapter falls back to the pre-fix "stop dead and wait for rotation that
# never comes" behaviour.
#
# Usage:  bash run_escape_ab.sh
# Output: $WS/ab_{off,on}_safety.json  (summaries), $WS/ab_{off,on}_verify.log
set -o pipefail
export LANG=C
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

YAML=/mnt/e/puppyfangzhen/src/puppy_minicpm_robot/config/minicpm_robot.yaml
VERIFY=/mnt/e/puppyfangzhen/scripts/_run_verify_wsl.sh
RECORD=/mnt/e/puppyfangzhen/tools/record_safety_live.py
WS="${WS:-$HOME/puppy_ws}"
OUTDIR="${OUTDIR:-$WS}"

for s in /opt/ros/humble/setup.bash "$WS/install/setup.bash"; do
  [[ -f "$s" ]] && source "$s"
done

cp "$YAML" "$WS/ab_yaml_backup.yaml"
restore() { cp "$WS/ab_yaml_backup.yaml" "$YAML"; echo "=== yaml restored ==="; }
trap restore EXIT

set_factor() {
  sed -i "s/^    escape_clearance_factor: .*/    escape_clearance_factor: $1/" "$YAML"
  echo -n "  yaml now: "
  grep "escape_clearance_factor" "$YAML"
}

run_arm() {
  local label="$1" factor="$2"
  echo "=== ARM '$label' (escape_clearance_factor=$factor) ==="
  set_factor "$factor"

  bash "$VERIFY" > "$OUTDIR/ab_${label}_verify.log" 2>&1 &
  local vpid=$!

  # Record for the WHOLE run, from the same instant the verification starts.
  # An earlier version slept 75 s and then recorded for 200 s on the assumption
  # that the mission would be dispatched around then. It is not: the standby
  # window issues 8 `ros2 topic echo --once` probes and each pays a full DDS
  # handshake, so it can run well past 150 s. That run recorded 3597 samples of
  # which every single one was STANDBY -- the mission it was meant to observe
  # completed after the recorder had already stopped. Cover everything and let
  # the report separate STANDBY from the navigation samples afterwards.
  python3 "$RECORD" --duration 480 --out "$OUTDIR/ab_${label}_safety.json" \
    > "$OUTDIR/ab_${label}_recorder.log" 2>&1 &
  local rpid=$!
  wait "$vpid"
  wait "$rpid"

  # verify_standby_live.sh writes to a fixed path, so each arm overwrites the
  # previous one -- copy it out before the next arm runs.
  cp "$WS/verify_standby.log" "$OUTDIR/ab_${label}_verify_full.log"

  echo "--- verify tail ($label) ---"
  tail -n 14 "$OUTDIR/ab_${label}_verify_full.log"
  echo "--- full-rate safety ($label) ---"
  python3 -c "
import json,sys
d=json.load(open('$OUTDIR/ab_${label}_safety.json'))
for k in ('samples','obstacle_distance_min','obstacle_distance_p05',
          'blocked_duration_max_s','lidar_override_samples',
          'blocked_escalated_samples','commanded_vy_nonzero'):
    print('  %-26s %s' % (k, d.get(k)))
print('  reasons:')
for r,c in sorted(d.get('reasons',{}).items(), key=lambda kv:-kv[1]):
    print('    %5d  %s' % (c, r[:90]))
"
}

run_arm off 999.0
run_arm on 1.15
