#!/usr/bin/env python3
"""Calibrated depth test: place camera at known distances, read depth."""
import math, struct
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

s = RemoteAPIClient().getObject("sim")

try:
    old = s.getObject("/patrol_camera")
    s.removeObject(old)
except:
    pass

vs = s.createVisionSensor(3, [64, 48, 0, 0], [0.01, 30.0, math.pi/3, 0.1, 0,0, 0.5,0.5,0.5, 0,0])
s.setObjectAlias(vs, "patrol_camera")

s.startSimulation()
import time; time.sleep(0.3)

# Robot starts near (1, -2). Wall_south is at y=-4, wall_east at x=5.
# Place camera at y=0, looking south (-Y direction, towards wall_south at y=-4)
# At y=0: distance to south wall = 4m
# At y=-1: distance = 3m
# At y=-2: distance = 2m
# At y=-3: distance = 1m

test_positions = [
    (0, 0, 4.0, "4m to south wall"),
    (0, -1, 3.0, "3m to south wall"),
    (0, -2, 2.0, "2m to south wall"),
    (0, -3, 1.0, "1m to south wall"),
    (0, -3.5, 0.5, "0.5m to south wall"),
]

for cam_x, cam_y, expected, label in test_positions:
    s.setObjectPosition(vs, -1, [cam_x, cam_y, 0.5])
    # Look south: heading = -pi/2 (negative Y)
    s.setObjectOrientation(vs, -1, [0, math.pi/2 + 0.2, -math.pi/2])
    s.handleVisionSensor(vs)
    result = s.getVisionSensorDepth(vs)
    raw = result[0]
    res = result[1]
    dw, dh = res[0], res[1]

    if isinstance(raw, (bytes, bytearray)):
        flat = list(struct.unpack(f'{len(raw)//4}f', bytes(raw)))
    else:
        flat = list(raw)

    # Center pixel
    cx, cy = dw//2, dh//2
    center_val = flat[cy * dw + cx]
    # Also min nonzero
    vals = [v for v in flat if v > 0.001]
    mn = min(vals) if vals else 0
    mx = max(vals) if vals else 0
    mean = sum(vals)/len(vals) if vals else 0

    # Try interpretations
    interp_linear = center_val  # if meters directly
    interp_norm = center_val * 30.0  # if normalized 0-1 * far_plane
    interp_inv = 1.0 / center_val if center_val > 0.001 else 0  # if inverse depth

    print(f"  {label}: center={center_val:.6f} min={mn:.4f} max={mx:.4f} mean={mean:.4f} | "
          f"linear={interp_linear:.3f}m norm={interp_norm:.2f}m inv={interp_inv:.2f}m")

s.stopSimulation()
time.sleep(0.5)
try:
    s.removeObject(vs)
except:
    pass
