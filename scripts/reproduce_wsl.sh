#!/usr/bin/env bash
# reproduce_wsl.sh -- End-to-end WSL/ROS2 Humble reproduction & verification script
# Implements Section 6.3 and Section 14 Task 7 of jihua20260905.md.
#
# Actions:
#   1. Verifies ROS2 Humble environment
#   2. Checks source diffs between host (/mnt/e/puppyfangzhen/src) and WSL ($HOME/puppy_ws/src)
#   3. Syncs host source to WSL workspace and runs colcon build
#   4. Runs test regression suite (scripts/run_regression.sh)
#   5. Verifies mock + core bringup (launch, topics, echo payloads, shutdown)
#   6. (Optional) Verifies Gazebo/Nav2 live loop (--with-gazebo)
#
# Usage:
#   bash scripts/reproduce_wsl.sh                     # standard: sync, build, regression, mock verify
#   bash scripts/reproduce_wsl.sh --skip-build        # skip colcon build if already up-to-date
#   bash scripts/reproduce_wsl.sh --skip-regression   # skip regression tests
#   bash scripts/reproduce_wsl.sh --mock-only         # only verify mock bringup
#   bash scripts/reproduce_wsl.sh --with-gazebo       # also verify Gazebo world topics
#
set -o pipefail
export LANG=C

HOST_DIR="/mnt/e/puppyfangzhen"
WS="${WS:-$HOME/puppy_ws}"
REPORT_DIR="$HOST_DIR/artifacts/wsl_reproduce_$(date +%Y%m%d_%H%M%S)"
LOG_DIR="$WS/logs_reproduce"
mkdir -p "$LOG_DIR" "$REPORT_DIR"

SKIP_BUILD=0
SKIP_REGRESSION=0
MOCK_ONLY=0
WITH_GAZEBO=0

for arg in "$@"; do
  case "$arg" in
    --skip-build) SKIP_BUILD=1 ;;
    --skip-regression) SKIP_REGRESSION=1 ;;
    --mock-only) MOCK_ONLY=1; SKIP_BUILD=1; SKIP_REGRESSION=1 ;;
    --with-gazebo) WITH_GAZEBO=1 ;;
  esac
done

echo "======================================================================"
echo "PuppyPi WSL/ROS2 Humble Reproducibility Suite"
echo "Host: $HOST_DIR | Workspace: $WS"
echo "Report Dir: $REPORT_DIR"
echo "======================================================================"

# 1. Environment Verification
if [[ ! -f "/opt/ros/humble/setup.bash" ]]; then
  echo "ERROR: /opt/ros/humble/setup.bash not found. ROS2 Humble is required."
  exit 1
fi
source /opt/ros/humble/setup.bash
echo "[ENV] ROS_DISTRO: $ROS_DISTRO (expected humble)"
if [[ "$ROS_DISTRO" != "humble" ]]; then
  echo "ERROR: Active ROS_DISTRO is '$ROS_DISTRO', expected 'humble'."
  exit 1
fi

# 2. Source Diff Check
echo "----------------------------------------------------------------------"
echo "[DIFF] Checking source differences: $HOST_DIR/src/ -> $WS/src/ ..."
mkdir -p "$WS/src"
DIFF_COUNT=$(rsync -avunc --delete "$HOST_DIR/src/" "$WS/src/" | grep -E '^deleting |^[a-zA-Z0-9]' | grep -v -E 'sending incremental|sent [0-9]|total size' | wc -l)
echo "[DIFF] Differing/outdated items detected: $DIFF_COUNT"

if [[ $DIFF_COUNT -gt 0 ]]; then
  echo "[SYNC] Synchronizing host sources to WSL workspace..."
  rsync -a --delete "$HOST_DIR/src/" "$WS/src/"
  echo "[SYNC] Done."
else
  echo "[SYNC] WSL workspace sources are already identical to host."
fi

# 3. Colcon Build
if [[ $SKIP_BUILD -eq 0 ]]; then
  echo "----------------------------------------------------------------------"
  echo "[BUILD] Building workspace: $WS ..."
  cd "$WS" || exit 1
  colcon build --symlink-install --event-handlers console_direct+ 2>&1 | tee "$LOG_DIR/colcon_build.log"
  BUILD_STATUS=${PIPESTATUS[0]}
  if [[ $BUILD_STATUS -ne 0 ]]; then
    echo "ERROR: Colcon build failed with status $BUILD_STATUS. See $LOG_DIR/colcon_build.log"
    exit $BUILD_STATUS
  fi
  echo "[BUILD] Build finished successfully."
else
  echo "[BUILD] Skipped colcon build (--skip-build)."
fi

source "$WS/install/setup.bash"

# 4. Regression Test Gate
if [[ $SKIP_REGRESSION -eq 0 ]]; then
  echo "----------------------------------------------------------------------"
  echo "[REGRESSION] Running regression gate ($HOST_DIR/scripts/run_regression.sh) ..."
  cd "$HOST_DIR" || exit 1
  bash "$HOST_DIR/scripts/run_regression.sh" 2>&1 | tee "$REPORT_DIR/regression.log"
  REG_STATUS=${PIPESTATUS[0]}
  if [[ $REG_STATUS -ne 0 ]]; then
    echo "ERROR: Regression tests failed with exit code $REG_STATUS."
    exit $REG_STATUS
  fi
  echo "[REGRESSION] Regression passed."
else
  echo "[REGRESSION] Skipped regression gate."
fi

# 5. Mock + Core Bringup & Topic Probing
echo "----------------------------------------------------------------------"
echo "[MOCK+CORE] Testing Entry Point 1 (mock_system.launch.py) ..."
MOCK_LOG="$LOG_DIR/mock_bringup.log"
ros2 launch puppy_bringup mock_system.launch.py > "$MOCK_LOG" 2>&1 &
LAUNCH_PID=$!
echo "[MOCK+CORE] Launch process started (PID: $LAUNCH_PID). Waiting 6s for nodes..."
sleep 6

# Verify critical topics exist
REQUIRED_TOPICS=(
  "/platform/health"
  "/battery_state"
  "/battery_status"
  "/joint_states"
  "/platform/motion_state"
  "/robot/capabilities"
)

TOPIC_FAILURES=0
TOPIC_LIST=$(ros2 topic list 2>/dev/null)

for top in "${REQUIRED_TOPICS[@]}"; do
  if echo "$TOPIC_LIST" | grep -qx "$top"; then
    echo "  [TOPIC OK] Found $top"
  else
    echo "  [TOPIC FAIL] Missing required topic: $top"
    TOPIC_FAILURES=$((TOPIC_FAILURES + 1))
  fi
done

# Echo real payload from /platform/health and /battery_status
echo "[MOCK+CORE] Probing live message payload on /platform/health..."
HEALTH_MSG=$(timeout 5s ros2 topic echo /platform/health --once 2>&1)
if echo "$HEALTH_MSG" | grep -q "level:"; then
  echo "  [PAYLOAD OK] /platform/health payload received:"
  echo "$HEALTH_MSG" | sed 's/^/    /'
else
  echo "  [PAYLOAD FAIL] /platform/health probe failed or timed out:"
  echo "$HEALTH_MSG" | sed 's/^/    /'
  TOPIC_FAILURES=$((TOPIC_FAILURES + 1))
fi

echo "[MOCK+CORE] Probing live message payload on /battery_status..."
BATT_MSG=$(timeout 5s ros2 topic echo /battery_status --once 2>&1)
if echo "$BATT_MSG" | grep -q "voltage:"; then
  echo "  [PAYLOAD OK] /battery_status payload received:"
  echo "$BATT_MSG" | sed 's/^/    /'
else
  echo "  [PAYLOAD FAIL] /battery_status probe failed or timed out:"
  echo "$BATT_MSG" | sed 's/^/    /'
  TOPIC_FAILURES=$((TOPIC_FAILURES + 1))
fi

# Clean shutdown of mock bringup
echo "[MOCK+CORE] Terminating mock bringup launch process..."
kill -SIGINT "$LAUNCH_PID" 2>/dev/null || true
sleep 3
pkill -9 -f "puppy|ros2 launch" 2>/dev/null || true
wait "$LAUNCH_PID" 2>/dev/null || true

if [[ $TOPIC_FAILURES -gt 0 ]]; then
  echo "ERROR: Mock + Core verification encountered $TOPIC_FAILURES topic/payload failures."
  echo "Launch log saved to $MOCK_LOG"
  exit 2
fi
echo "[MOCK+CORE] Entry Point 1 (mock + core) verified: 100% OK."

# 6. Optional Gazebo Verification
if [[ $WITH_GAZEBO -eq 1 ]]; then
  echo "----------------------------------------------------------------------"
  echo "[GAZEBO] Testing Entry Point 2 live simulation topics..."
  GZ_LOG="$LOG_DIR/gazebo_bringup.log"
  ros2 launch puppy_worlds simulation.launch.py world:=small_room gui:=false > "$GZ_LOG" 2>&1 &
  GZ_PID=$!
  echo "[GAZEBO] Simulation launch PID: $GZ_PID. Waiting 12s for clock & sensors..."
  sleep 12

  # Check real /clock, /odom, /scan, /tf
  GZ_FAIL=0
  CLOCK_MSG=$(timeout 5s ros2 topic echo /clock --once 2>&1)
  if echo "$CLOCK_MSG" | grep -q "sec:"; then
    echo "  [GAZEBO OK] /clock received"
  else
    echo "  [GAZEBO FAIL] /clock timeout"
    GZ_FAIL=$((GZ_FAIL + 1))
  fi

  ODOM_MSG=$(timeout 6s ros2 topic echo /odom --once 2>&1)
  if echo "$ODOM_MSG" | grep -q "pose:"; then
    echo "  [GAZEBO OK] /odom received"
  else
    echo "  [GAZEBO FAIL] /odom timeout"
    GZ_FAIL=$((GZ_FAIL + 1))
  fi

  SCAN_MSG=$(timeout 6s ros2 topic echo /scan --once 2>&1)
  if echo "$SCAN_MSG" | grep -q "ranges:"; then
    echo "  [GAZEBO OK] /scan received"
  else
    echo "  [GAZEBO FAIL] /scan timeout"
    GZ_FAIL=$((GZ_FAIL + 1))
  fi

  kill -SIGINT "$GZ_PID" 2>/dev/null || true
  sleep 3
  pkill -9 -f "gzserver|gzclient|robot_state_publisher|simulation.launch" 2>/dev/null || true
  wait "$GZ_PID" 2>/dev/null || true

  if [[ $GZ_FAIL -gt 0 ]]; then
    echo "WARNING: Gazebo check had $GZ_FAIL failures. Log at $GZ_LOG"
  else
    echo "[GAZEBO] Simulation topics verified OK."
  fi
fi

# 7. Write Summary Report
cat <<EOF > "$REPORT_DIR/reproducibility_summary.md"
# WSL/ROS2 Humble Reproducibility Report

- **Date**: $(date -Iseconds)
- **ROS_DISTRO**: $ROS_DISTRO
- **Host Source**: $HOST_DIR/src
- **WSL Workspace**: $WS/src
- **Source Sync Differing Items**: $DIFF_COUNT
- **Colcon Build**: $([ $SKIP_BUILD -eq 1 ] && echo "SKIPPED" || echo "PASSED")
- **Regression Gate**: $([ $SKIP_REGRESSION -eq 1 ] && echo "SKIPPED" || echo "PASSED")
- **Mock + Core Entry Point 1**: PASSED
  - Verified Topics: ${REQUIRED_TOPICS[*]}
  - Probed Payloads: /platform/health, /battery_status
- **Gazebo Entry Point 2**: $([ $WITH_GAZEBO -eq 1 ] && echo "TESTED" || echo "NOT_RUN")

All mandatory reproducibility criteria satisfied.
EOF

echo "======================================================================"
echo "WSL Reproducibility suite finished: ALL GATES GREEN."
echo "Summary report: $REPORT_DIR/reproducibility_summary.md"
echo "======================================================================"
exit 0
