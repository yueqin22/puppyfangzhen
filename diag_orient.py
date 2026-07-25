#!/usr/bin/env python3
"""Diagnose vision sensor orientation. Tests multiple Euler angle conventions
to find which one correctly points the vision sensor at the scene."""
import math, sys, os
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from PIL import Image

print("Connecting...", flush=True)
client = RemoteAPIClient()
sim = client.getObject("sim")
print("Connected.", flush=True)

# Stop any running sim
try:
    sim.stopSimulation()
    import time; time.sleep(1)
except:
    pass

# Get robot position
base = sim.getObject("/base_footprint")
rpos = sim.getObjectPosition(base, -1)
print(f"Robot pos: {rpos}", flush=True)

# Create vision sensor
try:
    old = sim.getObject("/diag_camera")
    sim.removeObject(old)
except:
    pass

angle_rad = 60.0 * math.pi / 180.0
vs = sim.createVisionSensor(3, [640, 480, 0, 0],
    [0.01, 50.0, angle_rad, 0.1, 0, 0, 0.5, 0.5, 0.5, 0, 0])
sim.setObjectAlias(vs, "diag_camera")
print(f"Created vs: {vs}", flush=True)

def render_and_check(orient, fname, label):
    sim.setObjectOrientation(vs, -1, orient)
    sim.handleVisionSensor(vs)
    try:
        result = sim.getVisionSensorImg(vs)
    except Exception:
        result = sim.getVisionSensorImage(vs)
    if isinstance(result, (list, tuple)) and len(result) >= 2:
        img_data = result[0]
        res = result[1]
        if img_data and res and len(res) >= 2:
            w, h = res[0], res[1]
            img = Image.frombytes("RGB", (w, h), img_data)
            img = img.transpose(Image.FLIP_TOP_BOTTOM)
            img.save(fname, format="JPEG", quality=75)
            ex = img.getextrema()
            black = all((isinstance(e, tuple) and e[0]==0 and e[1]==0) or e==0 for e in ex)
            print(f"  {label}: orient={[round(x,3) for x in orient]} -> {fname} extrema={ex} {'BLACK' if black else 'HAS_CONTENT'}", flush=True)
            return not black
    print(f"  {label}: NO DATA", flush=True)
    return False

# Test from directly above (diag_vs.py verified [pi,0,0] works)
print("\n== Test A: directly above, [pi,0,0] (verified by diag_vs.py) ==", flush=True)
sim.setObjectPosition(vs, -1, [rpos[0], rpos[1], 8.0])
render_and_check([math.pi, 0, 0], "/tmp/diag_a.jpg", "A_pi_0_0")

# Test from behind-above (cam at robot - 4m behind, height 3.5)
cam_x = rpos[0] - 4.0
cam_y = rpos[1]
cam_z = 3.5
sim.setObjectPosition(vs, -1, [cam_x, cam_y, cam_z])
dx = rpos[0] - cam_x  # = 4
dy = rpos[1] - cam_y  # = 0
dz = 0.3 - cam_z       # = -3.2
d = math.sqrt(dx*dx + dy*dy + dz*dz)
horiz = math.sqrt(dx*dx + dy*dy)

print(f"\n== Test B: behind-above, cam=({cam_x:.2f},{cam_y:.2f},{cam_z}), target=({rpos[0]:.2f},{rpos[1]:.2f},0.3) ==", flush=True)
print(f"  dx={dx}, dy={dy}, dz={dz}, d={d:.3f}, horiz={horiz:.3f}", flush=True)

# B1: my derived formula (X-Y-Z intrinsic => R = Rz(yaw)*Rx(pitch))
pitch_b1 = math.acos(dz/d) if d>1e-4 else 0
yaw_b1 = math.atan2(dx, -dy) if horiz>1e-4 else 0
render_and_check([pitch_b1, 0, yaw_b1], "/tmp/diag_b1.jpg", f"B1_derived pitch={pitch_b1:.3f} yaw={yaw_b1:.3f}")

# B2: original code formula
pitch_b2 = math.atan2(-dz, horiz)
yaw_b2 = math.atan2(dy, dx)
render_and_check([pitch_b2, 0, yaw_b2], "/tmp/diag_b2.jpg", f"B2_original pitch={pitch_b2:.3f} yaw={yaw_b2:.3f}")

# B3: maybe CoppeliaSim uses Z-Y-X (gamma=roll, beta=pitch, alpha=yaw) => [yaw, pitch, 0]
# Try treating [alpha,beta,gamma] as [yaw, pitch, roll] with Z-Y-X intrinsic
# R = Rx(gamma)*Ry(beta)*Rz(alpha), view (0,0,1)
# Let's just try common variants
render_and_check([0, pitch_b1, yaw_b1], "/tmp/diag_b3.jpg", f"B3_[0,pitch,yaw]")

# B4: pure downward + yaw
render_and_check([math.pi, 0, yaw_b1], "/tmp/diag_b4.jpg", f"B4_[pi,0,yaw={yaw_b1:.3f}]")

# B5: half rotation variants
render_and_check([math.pi/2, 0, yaw_b1], "/tmp/diag_b5.jpg", f"B5_[pi/2,0,yaw={yaw_b1:.3f}]")

# B6: try using setObjectMatrix directly (look-at matrix)
print("\n== Test C: setObjectMatrix look-at ==", flush=True)
fx, fy, fz = dx/d, dy/d, dz/d
# right = forward x world_up
rx_ = fy*1 - fz*0
ry_ = fz*0 - fx*1
rz_ = fx*0 - fy*0
r = math.sqrt(rx_*rx_ + ry_*ry_ + rz_*rz_)
if r < 1e-4:
    rx_, ry_, rz_ = 0, 1, 0; r = 1
rx_, ry_, rz_ = rx_/r, ry_/r, rz_/r
# up = right x forward
ux = ry_*fz - rz_*fy
uy = rz_*fx - rx_*fz
uz = rx_*fy - ry_*fx
# Matrix 3x4 row-major: columns are X(right), Y(up), Z(forward)
matrix = [rx_, ux, fx, cam_x,
          ry_, uy, fy, cam_y,
          rz_, uz, fz, cam_z]
print(f"  matrix={[round(x,3) for x in matrix]}", flush=True)
try:
    sim.setObjectMatrix(vs, -1, matrix)
    sim.handleVisionSensor(vs)
    try:
        result = sim.getVisionSensorImg(vs)
    except Exception:
        result = sim.getVisionSensorImage(vs)
    if isinstance(result, (list, tuple)) and len(result) >= 2:
        img_data = result[0]; res = result[1]
        if img_data and res and len(res) >= 2:
            w, h = res[0], res[1]
            img = Image.frombytes("RGB", (w, h), img_data)
            img = img.transpose(Image.FLIP_TOP_BOTTOM)
            img.save("/tmp/diag_c.jpg", format="JPEG", quality=75)
            ex = img.getextrema()
            black = all((isinstance(e, tuple) and e[0]==0 and e[1]==0) or e==0 for e in ex)
            print(f"  C_matrix -> /tmp/diag_c.jpg extrema={ex} {'BLACK' if black else 'HAS_CONTENT'}", flush=True)
except Exception as e:
    print(f"  C_matrix ERROR: {e}", flush=True)

# Cleanup
sim.removeObject(vs)
print("\nDone.", flush=True)
