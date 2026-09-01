#!/bin/bash
export PATH="/home/veni/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
source /opt/ros/humble/setup.bash
cd ~/puppy_ws
colcon build --symlink-install --packages-select puppy_bringup 2>&1 | tail -15
