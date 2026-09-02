#!/usr/bin/env bash
# One-shot launcher: run the live verification from a freshly restarted world.
# Kept as a file because `wsl -c` does not expand shell variables reliably; the
# verification script itself redirects everything to $WS/verify_standby.log.
set +u
cd "$HOME/puppy_ws" || exit 2
bash /mnt/e/puppyfangzhen/src/puppy_minicpm_robot/scripts/verify_standby_live.sh \
  --mission --restart-world
echo "RUN_EXIT=$?"
