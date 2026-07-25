#!/usr/bin/env python3
import os, sys, time
os.environ['FASTRTPS_DEFAULT_PROFILES_FILE'] = '/home/veni/fastdds_profile.xml'
os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
os.environ['ROS_DOMAIN_ID'] = '0'
os.environ['ROS_LOCALHOST_ONLY'] = '1'

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from sensor_msgs.msg import BatteryState, LaserScan
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock

class VerifyNode(Node):
    def __init__(self):
        super().__init__('verify_node')
        self.results = {}
        topics = [
            ('battery', '/battery_state', BatteryState),
            ('odom', '/odom', Odometry),
            ('scan', '/scan', LaserScan),
            ('clock', '/clock', Clock),
            ('low_battery', '/low_battery_alert', Bool),
            ('fall_detected', '/fall_detected', Bool),
            ('emergency', '/emergency/status', String),
        ]
        for key, topic, msg_type in topics:
            self.results[key] = None
            self.create_subscription(
                msg_type, topic,
                lambda msg, k=key: self.cb(k, msg), 10)

    def cb(self, key, msg):
        if self.results[key] is None:
            self.results[key] = msg

def main():
    rclpy.init()
    node = VerifyNode()
    deadline = time.time() + 10
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.5)
        received = sum(1 for v in node.results.values() if v is not None)
        if received >= 6:
            break

    print("=== Verification Results ===")
    print()

    r = node.results
    if r['clock']:
        t = r['clock'].clock
        print(f"[OK] /clock: sim_time={t.sec}.{t.nanosec//1000000:03d}s")
    else:
        print("[FAIL] /clock")

    if r['battery']:
        b = r['battery']
        st = "CHARGING" if b.power_supply_status == 1 else "DISCHARGING"
        print(f"[OK] /battery_state: {b.percentage*100:.1f}% {st} {b.voltage:.1f}V")
    else:
        print("[FAIL] /battery_state")

    if r['low_battery'] is not None:
        print(f"[OK] /low_battery_alert: {r['low_battery'].data}")
    else:
        print("[FAIL] /low_battery_alert")

    if r['fall_detected'] is not None:
        fd = r['fall_detected'].data
        ok = "OK" if fd == False else "WARN"
        print(f"[{ok}] /fall_detected: {fd} (expect False)")
    else:
        print("[FAIL] /fall_detected")

    if r['emergency'] is not None:
        print(f"[OK] /emergency/status: '{r['emergency'].data}'")
    else:
        print("[FAIL] /emergency/status")

    if r['odom']:
        o = r['odom']
        x = o.pose.pose.position.x
        y = o.pose.pose.position.y
        vx = o.twist.twist.linear.x
        print(f"[OK] /odom: pos=({x:.2f},{y:.2f}) vx={vx:.3f}")
    else:
        print("[FAIL] /odom")

    if r['scan']:
        s = r['scan']
        valid = sum(1 for d in s.ranges if s.range_min < d < s.range_max)
        print(f"[OK] /scan: {len(s.ranges)} rays {valid} valid")
    else:
        print("[FAIL] /scan")

    print()
    print("=== Nodes ===")
    names = sorted(node.get_node_names())
    for n in names:
        print(f"  /{n}")
    print(f"  Total: {len(names)}")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
