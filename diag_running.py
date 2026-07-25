#!/usr/bin/env python3
"""Test if vision sensor works DURING simulation (after startSimulation)."""
import math, time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from PIL import Image

client = RemoteAPIClient()
sim = client.getObject("sim")

try:
    sim.stopSimulation(); time.sleep(1)
except: pass

base = sim.getObject("/base_footprint")
rpos = sim.getObjectPosition(base, -1)
print(f"Robot pos (stopped): {rpos}", flush=True)

try:
    old = sim.getObject("/diag2"); sim.removeObject(old)
except: pass

angle_rad = 60.0 * math.pi / 180.0
vs = sim.createVisionSensor(3, [640, 480, 0, 0],
    [0.01, 50.0, angle_rad, 0.1, 0, 0, 0.5, 0.5, 0.5, 0, 0])
sim.setObjectAlias(vs, "diag2")
print(f"Created vs: {vs}", flush=True)

def render(label, fname):
    sim.handleVisionSensor(vs)
    try:
        result = sim.getVisionSensorImg(vs)
    except Exception:
        result = sim.getVisionSensorImage(vs)
    if isinstance(result, (list, tuple)) and len(result) >= 2:
        img_data, res = result[0], result[1]
        if img_data and res and len(res) >= 2:
            w, h = res[0], res[1]
            img = Image.frombytes("RGB", (w, h), img_data)
            img = img.transpose(Image.FLIP_TOP_BOTTOM)
            img.save(fname, format="JPEG", quality=75)
            ex = img.getextrema()
            black = all((isinstance(e, tuple) and e[0]==0 and e[1]==0) or e==0 for e in ex)
            print(f"  {label}: {fname} extrema={ex} {'BLACK' if black else 'HAS_CONTENT'}", flush=True)
            return not black
    print(f"  {label}: NO DATA", flush=True)
    return False

cam_x = rpos[0] - 4.0
cam_y = rpos[1]
cam_z = 3.5
dx, dy, dz = rpos[0]-cam_x, rpos[1]-cam_y, 0.3-cam_z
d = math.sqrt(dx*dx+dy*dy+dz*dz)
pitch = math.acos(dz/d)
yaw = math.atan2(dx, -dy) if (dx*dx+dy*dy)>1e-4 else 0
print(f"cam=({cam_x:.2f},{cam_y:.2f},{cam_z}) pitch={pitch:.3f} yaw={yaw:.3f}", flush=True)

# Test 1: before startSimulation
print("\n== Test 1: BEFORE startSimulation ==", flush=True)
sim.setObjectPosition(vs, -1, [cam_x, cam_y, cam_z])
sim.setObjectOrientation(vs, -1, [pitch, 0, yaw])
render("T1_stopped", "/tmp/diag2_t1.jpg")

# Test 2: after startSimulation
print("\n== Test 2: AFTER startSimulation ==", flush=True)
sim.startSimulation()
time.sleep(0.5)
sim.setObjectPosition(vs, -1, [cam_x, cam_y, cam_z])
sim.setObjectOrientation(vs, -1, [pitch, 0, yaw])
render("T2_running", "/tmp/diag2_t2.jpg")

# Test 3: re-set position after start (maybe start resets it)
print("\n== Test 3: set pos again after start, render ==", flush=True)
sim.setObjectPosition(vs, -1, [cam_x, cam_y, cam_z])
sim.setObjectOrientation(vs, -1, [pitch, 0, yaw])
rnow = sim.getObjectPosition(vs, -1)
onow = sim.getObjectOrientation(vs, -1)
print(f"  vs pos now: {rnow}, ori now: {onow}", flush=True)
render("T3_reset", "/tmp/diag2_t3.jpg")

# Test 4: pure top-down while running
print("\n== Test 4: top-down [pi,0,0] while running ==", flush=True)
sim.setObjectPosition(vs, -1, [rpos[0], rpos[1], 8.0])
sim.setObjectOrientation(vs, -1, [math.pi, 0, 0])
render("T4_topdown_running", "/tmp/diag2_t4.jpg")

sim.stopSimulation()
time.sleep(1)
sim.removeObject(vs)
print("\nDone.", flush=True)
