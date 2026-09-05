#!/usr/bin/env bash
# Print the robot's ground-truth pose (z in particular) from /model_states.
# Kept as a file because inline python quoting inside `wsl bash -c '...'` is
# eaten by the outer shell.
set +u
export LANG=C
WS="${WS:-$HOME/puppy_ws}"
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
python3 - <<'PY'
import math, time
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates

class P(Node):
    def __init__(self):
        super().__init__("read_pose")
        self.msg = None
        self.create_subscription(ModelStates, "/model_states", self.cb, 10)
    def cb(self, m):
        self.msg = m

rclpy.init()
n = P()
end = time.time() + 20.0
while time.time() < end and n.msg is None:
    rclpy.spin_once(n, timeout_sec=0.3)
if n.msg is None:
    print("no /model_states")
else:
    for i, name in enumerate(n.msg.name):
        if name != "puppy":
            continue
        p = n.msg.pose[i].position
        q = n.msg.pose[i].orientation
        yaw = math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
        print("puppy: x=%+.4f y=%+.4f z=%+.4f yaw=%+.2f deg" % (p.x, p.y, p.z, yaw))
rclpy.shutdown()
PY
