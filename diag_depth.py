#!/usr/bin/env python3
"""Diagnose vision sensor depth format: sample many points + check range."""
import math, struct
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from PIL import Image

s = RemoteAPIClient().getObject("sim")
base = s.getObject("/base_footprint")

# Create vision sensor
try:
    old = s.getObject("/patrol_camera")
    s.removeObject(old)
except:
    pass

vs = s.createVisionSensor(3, [320,240,0,0], [0.01, 30.0, math.pi/3, 0.1, 0,0, 0.5,0.5,0.5, 0,0])
s.setObjectAlias(vs, "patrol_camera")

s.startSimulation()
import time; time.sleep(0.5)

pos = s.getObjectPosition(base, -1)
# Place camera high, looking forward and down
s.setObjectPosition(vs, -1, [pos[0]+0.25, pos[1], 1.0])
s.setObjectOrientation(vs, -1, [0, math.pi/2 + 0.2, 0])  # 11 deg down

s.handleVisionSensor(vs)
result = s.getVisionSensorDepth(vs)
raw = result[0]
res = result[1]
dw, dh = res[0], res[1]
print(f"Resolution: {dw}x{dh}  raw type={type(raw).__name__} len={len(raw)}")

# Unpack
if isinstance(raw, (bytes, bytearray)):
    n = len(raw) // 4
    flat = list(struct.unpack(f'{n}f', bytes(raw)))
else:
    flat = list(raw)

print(f"Unpacked: {len(flat)} floats")

# Find min/max/mean
vals = [v for v in flat if v != 0.0]
if vals:
    print(f"Nonzero count: {len(vals)}/{len(flat)}")
    print(f"Min={min(vals):.6f}  Max={max(vals):.6f}  Mean={sum(vals)/len(vals):.6f}")
    # Sample grid: 5x5 across image
    print("\nDepth grid (5x5 sampled across image, row 0=top):")
    for gy in range(5):
        row = []
        for gx in range(5):
            x = int(dw * (gx+0.5)/5)
            y = int(dh * (gy+0.5)/5)
            idx = y * dw + x
            v = flat[idx] if idx < len(flat) else -1
            row.append(f"{v:.4f}")
        print(f"  y={int(dh*(gy+0.5)/5):3d}: {' '.join(row)}")

    # Also check: are values in 0-1 range?
    in_range = sum(1 for v in vals if 0 <= v <= 1.0)
    print(f"\nValues in [0,1]: {in_range}/{len(vals)}")
    if in_range == len(vals):
        print("=> Depth is NORMALIZED (0-1). Scale by far_plane=30: ")
        print(f"   Min real = {min(vals)*30:.2f}m  Max real = {max(vals)*30:.2f}m")
    else:
        print("=> Depth might be in METERS directly")
        print(f"   Min = {min(vals):.3f}m  Max = {max(vals):.3f}m")

# Also get the RGB to see what camera sees
img_result = s.getVisionSensorImg(vs)
if isinstance(img_result, (list, tuple)) and len(img_result) >= 2:
    img_data = img_result[0]
    ir = img_result[1]
    if img_data and ir:
        w, h = ir[0], ir[1]
        img = Image.frombytes("RGB", (w, h), img_data)
        img = img.transpose(Image.FLIP_TOP_BOTTOM)
        img.save("/tmp/depth_diag.jpg", format="JPEG", quality=85)
        ex = img.getextrema()
        black = all((isinstance(e, tuple) and e[0]==0 and e[1]==0) or e==0 for e in ex)
        print(f"\nRGB: {'BLACK' if black else 'HAS_CONTENT'} extrema={ex}")

s.stopSimulation()
time.sleep(0.5)
try:
    s.removeObject(vs)
except:
    pass
