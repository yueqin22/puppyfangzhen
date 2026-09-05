#!/usr/bin/env bash
# P0 end-to-end mission regression: rounds 2 and 3 under the hover config.
# Written to a FILE on purpose: `wsl -c '...'` eats `$r`/`$i` loop variables
# (the outer shell consumes them even inside single quotes), which silently
# collapsed a previous loop into a single run overwriting one log.
#
# The verification script redirects its own output to $WS/verify_standby.log,
# so it cannot be redirected here -- instead copy that log out after each round.
set +u
cd "$HOME/puppy_ws" || exit 2
for r in r2 r3; do
  echo "=== round $r starting ==="
  bash /mnt/e/puppyfangzhen/src/puppy_minicpm_robot/scripts/verify_standby_live.sh \
    --mission --restart-world
  echo "RUN_EXIT=$?" > "/mnt/e/puppyfangzhen/_p0_$r.log"
  cp "$HOME/puppy_ws/verify_standby.log" "/mnt/e/puppyfangzhen/_p0_${r}_detail.log"
  echo "=== round $r done ==="
done
echo P0_RERUN_DONE
