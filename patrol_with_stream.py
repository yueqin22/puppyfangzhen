#!/usr/bin/env python3
"""
Patrol + Gait + Vision Sensor Stream
Integrates robot patrol, leg animation, and vision sensor rendering into one script.
The vision sensor renders CoppeliaSim's 3D scene directly (bypassing X11), saving
JPEG frames to /tmp/stream_frame.jpg for the MJPEG server to pick up.
"""
import time, math, io
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from PIL import Image

client = RemoteAPIClient()
sim = client.getObject("sim")

# Get robot parts
base = sim.getObject("/base_footprint")
base_link = sim.getObject("/base_link")

legs = {}
for name in ["fl", "fr", "rl", "rr"]:
    legs[name] = {
        "upper": sim.getObject(f"/leg_{name}_upper"),
        "lower": sim.getObject(f"/leg_{name}_lower"),
        "foot":  sim.getObject(f"/leg_{name}_foot"),
    }

print("Robot parts loaded")

# --- Wall geometry for collision detection (manual, based on room layout) ---
# Room is 10m x 8m: x in [-5,5], y in [-4,4]
WALL_SEGMENTS = [
    ((-5, -4), (5, -4)),   # south wall
    ((-5, 4), (5, 4)),      # north wall
    ((-5, -4), (-5, 4)),    # west wall
    ((5, -4), (5, 4)),      # east wall
    ((-3, -4), (-3, 4)),    # divide_1
    ((3, -4), (3, 4)),      # divide_2
]
# Furniture as circular obstacles (cx, cy, radius)
FURNITURE_CIRCLES = [
    (3.5, -3, 0.6),   # sofa
    (-3.5, 3, 0.6),   # bed
    (2.0, 2.5, 0.6),  # dining_table
]
print(f"Walls: {len(WALL_SEGMENTS)} segments, Furniture: {len(FURNITURE_CIRCLES)}")


# --- Create Vision Sensor ---
try:
    old_vs = sim.getObject("/patrol_camera")
    sim.removeObject(old_vs)
    print(f"Removed old vision sensor: {old_vs}")
except:
    pass

angle_rad = 60.0 * math.pi / 180.0
vs_handle = sim.createVisionSensor(
    3,  # options: bit0=explicitly handled, bit1=perspective mode
    [640, 480, 0, 0],
    [0.01, 50.0, angle_rad, 0.1, 0, 0, 0.5, 0.5, 0.5, 0, 0]
)
sim.setObjectAlias(vs_handle, "patrol_camera")
print(f"Created vision sensor: handle={vs_handle}")

# Initial position (will be updated by update_vision_sensor)
sim.setObjectPosition(vs_handle, -1, [3.0, -3.0, 3.0])
sim.setObjectOrientation(vs_handle, -1, [math.pi, 0, 0])


def _ccw(A, B, C):
    return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])


def _seg_intersect(p1, p2, p3, p4):
    """True if segment p1p2 crosses segment p3p4."""
    return _ccw(p1, p3, p4) != _ccw(p2, p3, p4) and _ccw(p1, p2, p3) != _ccw(p1, p2, p4)


def _point_seg_dist(px, py, x1, y1, x2, y2):
    dx, dy = x2 - x1, y2 - y1
    if dx == 0 and dy == 0:
        return math.sqrt((px - x1) ** 2 + (py - y1) ** 2)
    t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)))
    return math.sqrt((px - (x1 + t * dx)) ** 2 + (py - (y1 + t * dy)) ** 2)


def check_path_clear(x1, y1, x2, y2, margin=0.4):
    """Return True if path (x1,y1)->(x2,y2) doesn't cross any wall/furniture."""
    for (wx1, wy1), (wx2, wy2) in WALL_SEGMENTS:
        if _seg_intersect((x1, y1), (x2, y2), (wx1, wy1), (wx2, wy2)):
            return False
        if _point_seg_dist(x2, y2, wx1, wy1, wx2, wy2) < margin:
            return False
    for cx, cy, r in FURNITURE_CIRCLES:
        if _point_seg_dist(cx, cy, x1, y1, x2, y2) < r + margin:
            return False
    if abs(x2) > 4.6 or abs(y2) > 3.6:
        return False
    return True


def animate_legs(t, phase_offset, moving):
    if not moving:
        for name in legs:
            sim.setObjectOrientation(legs[name]["upper"], base_link, [0, 0, 0])
            sim.setObjectOrientation(legs[name]["lower"], base_link, [0, 0, 0])
        return
    for name in legs:
        phase = phase_offset[name]
        cycle = (t * 4.0 + phase) % (2 * math.pi)
        hip_angle = 0.3 * math.sin(cycle)
        if 0 < cycle < math.pi:
            knee_angle = 0.5 * math.sin(cycle)
        else:
            knee_angle = 0
        sim.setObjectOrientation(legs[name]["upper"], base_link, [0, hip_angle, 0])
        sim.setObjectOrientation(legs[name]["lower"], base_link, [0, knee_angle, 0])


def update_vision_sensor(rx, ry, rangle):
    """Position vision sensor behind and above robot, look at robot, render frame.

    CoppeliaSim vision sensor default view direction is +Z (up). Euler angles
    [alpha, beta, gamma] use X-Y-Z intrinsic order. Given camera at cam_pos and
    target at robot pos, the forward vector is (dx, dy, dz) with dz<0 (target below).
    Solving R = Rz(yaw) * Rx(pitch) applied to (0,0,1):
        cos(pitch) = dz / |d|      (pitch > pi/2 since dz<0)
        sin(yaw)   = dx / horizontal
        cos(yaw)   = -dy / horizontal
        => pitch = acos(dz/|d|),  yaw = atan2(dx, -dy)
    Verified against diag_vs.py: cam at (0,0,8) looking down gives [pi, 0, 0].
    """
    cam_dist = 2.5
    cam_height = 4.0
    cam_x = rx - math.cos(rangle) * cam_dist
    cam_y = ry - math.sin(rangle) * cam_dist
    # Clamp camera inside room bounds so it never ends up beyond walls
    cam_x = max(-4.0, min(4.0, cam_x))
    cam_y = max(-3.0, min(3.0, cam_y))
    sim.setObjectPosition(vs_handle, -1, [cam_x, cam_y, cam_height])

    dx = rx - cam_x
    dy = ry - cam_y
    dz = 0.3 - cam_height  # negative: robot is below camera
    d = math.sqrt(dx * dx + dy * dy + dz * dz)
    horizontal = math.sqrt(dx * dx + dy * dy)

    if d > 1e-4:
        # Clamp for numerical safety; dz<0 => pitch in (pi/2, pi]
        pitch = math.acos(max(-1.0, min(1.0, dz / d)))
    else:
        pitch = 0.0
    yaw = math.atan2(dx, -dy) if horizontal > 1e-4 else 0.0

    sim.setObjectOrientation(vs_handle, -1, [pitch, 0, yaw])

    # DEBUG: verify actual pose
    actual_pos = sim.getObjectPosition(vs_handle, -1)
    actual_ori = sim.getObjectOrientation(vs_handle, -1)

    # Render and capture
    sim.handleVisionSensor(vs_handle)
    try:
        result = sim.getVisionSensorImg(vs_handle)
    except Exception:
        result = sim.getVisionSensorImage(vs_handle)

    if isinstance(result, (list, tuple)) and len(result) >= 2:
        img_data = result[0]
        res = result[1]
        if img_data and res and len(res) >= 2:
            try:
                w, h = res[0], res[1]
                img = Image.frombytes("RGB", (w, h), img_data)
                img = img.transpose(Image.FLIP_TOP_BOTTOM)
                img.save("/tmp/stream_frame.jpg", format="JPEG", quality=75)
                ex = img.getextrema()
                black = all((isinstance(e, tuple) and e[0]==0 and e[1]==0) or e==0 for e in ex)
                print(f"[VS] target=({rx:.2f},{ry:.2f},{rangle:.2f}) cam=({cam_x:.2f},{cam_y:.2f},{cam_height}) pitch={pitch:.3f} yaw={yaw:.3f} actual_pos={[round(v,2) for v in actual_pos]} actual_ori={[round(v,3) for v in actual_ori]} -> {'BLACK' if black else 'OK'}", flush=True)
            except Exception as e:
                print(f"Image save error: {e}")


# --- Patrol waypoints ---
waypoints = [
    (1.5, -2.0),
    (1.5, 1.0),
    (-1.0, 1.0),
    (-1.0, -2.0),
    (1.0, -2.0),
]
phase_offsets = {"fl": 0, "fr": math.pi, "rl": math.pi, "rr": 0}

# Start simulation
print("Starting simulation...")
sim.startSimulation()
time.sleep(0.5)

speed = 0.3
wp_idx = 0
t = 0
loop_count = 0
frame_count = 0
orig_pos = sim.getObjectPosition(base, -1)

# Initial frame capture
update_vision_sensor(orig_pos[0], orig_pos[1], 0)
print(f"Starting patrol: {len(waypoints)} waypoints")
try:
    while True:
        target = waypoints[wp_idx]
        curr = sim.getObjectPosition(base, -1)
        curr_ori = sim.getObjectOrientation(base, -1)

        dx = target[0] - curr[0]
        dy = target[1] - curr[1]
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < 0.2:
            wp_idx = (wp_idx + 1) % len(waypoints)
            if wp_idx == 0:
                loop_count += 1
                print(f"=== Loop {loop_count} completed ===")
            print(f"Waypoint {wp_idx}: target={waypoints[wp_idx]}")
            time.sleep(0.1)
            continue

        desired_angle = math.atan2(dy, dx)
        angle_diff = desired_angle - curr_ori[2]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        step = min(speed * 0.1, dist)
        new_x = curr[0] + step * math.cos(curr_ori[2])
        new_y = curr[1] + step * math.sin(curr_ori[2])

        # Check if the path to new position is clear of walls/furniture
        if not check_path_clear(curr[0], curr[1], new_x, new_y):
            print(f"Path blocked! Turning left...")
            curr_ori[2] += 0.3
            sim.setObjectOrientation(base, -1, curr_ori)
            animate_legs(t, phase_offsets, True)
            if frame_count % 5 == 0:
                update_vision_sensor(curr[0], curr[1], curr_ori[2])
            t += 0.1
            frame_count += 1
            time.sleep(0.1)
            continue

        if abs(angle_diff) > 0.1:
            curr_ori[2] += 0.1 * (1 if angle_diff > 0 else -1)

        sim.setObjectPosition(base, -1, [new_x, new_y, orig_pos[2]])
        sim.setObjectOrientation(base, -1, curr_ori)

        animate_legs(t, phase_offsets, True)

        if frame_count % 5 == 0:
            update_vision_sensor(new_x, new_y, curr_ori[2])

        t += 0.1
        frame_count += 1

        if frame_count % 50 == 0:
            print(f"  pos=({new_x:.2f}, {new_y:.2f}) wp={wp_idx} dist={dist:.2f} frames={frame_count}")

        time.sleep(0.1)

except KeyboardInterrupt:
    print("Stopped by user")
except Exception as e:
    print(f"Error: {e}")
    import traceback
    traceback.print_exc()
finally:
    print("Stopping simulation...")
    sim.stopSimulation()
    time.sleep(1)
    sim.setObjectPosition(base, -1, orig_pos)
    sim.setObjectOrientation(base, -1, [0, 0, 0])
    for name in legs:
        sim.setObjectOrientation(legs[name]["upper"], base_link, [0, 0, 0])
        sim.setObjectOrientation(legs[name]["lower"], base_link, [0, 0, 0])
    try:
        sim.removeObject(vs_handle)
    except:
        pass
    print("Done")
