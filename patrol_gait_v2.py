import time, math, subprocess, os
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

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
    for part in ["upper", "lower", "foot"]:
        h = legs[name][part]
        legs[name][part + "_pos"] = sim.getObjectPosition(h, base_link)
        legs[name][part + "_orient"] = sim.getObjectOrientation(h, base_link)

print("Robot parts loaded")

# Get walls for obstacle avoidance
walls = []
for obj in sim.getObjectsInTree(sim.handle_scene):
    try:
        alias = sim.getObjectAlias(obj)
        if any(k in alias for k in ["wall_", "sofa", "bed", "dining_table"]):
            pos = sim.getObjectPosition(obj, -1)
            walls.append((alias, pos[0], pos[1]))
    except:
        pass
print(f"Obstacles: {len(walls)}")

# Set camera initial position
cam = sim.getObject("/DefaultCamera")

def trigger_redraw():
    """Trigger CoppeliaSim 3D viewport redraw via xdotool mouse events"""
    try:
        env = {"DISPLAY": ":99", "PATH": "/usr/bin:/bin:/usr/local/bin"}
        subprocess.run(["xdotool", "mousemove", "400", "300"], env=env, capture_output=True, timeout=1)
        subprocess.run(["xdotool", "mousemove", "401", "301"], env=env, capture_output=True, timeout=1)
    except Exception:
        pass

def update_camera(rx, ry, rangle):
    """Camera follows robot from behind and above"""
    cam_dist = 3.5
    cam_height = 3.0
    # Camera positioned behind robot (opposite to heading direction)
    cam_x = rx - math.cos(rangle) * cam_dist
    cam_y = ry - math.sin(rangle) * cam_dist
    sim.setObjectPosition(cam, -1, [cam_x, cam_y, cam_height])
    # Look at robot
    dx = rx - cam_x
    dy = ry - cam_y
    dz = 0.3 - cam_height
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(-dz, math.sqrt(dx*dx + dy*dy))
    sim.setObjectOrientation(cam, -1, [pitch, 0, yaw])

# Start simulation
print("Starting simulation...")
sim.startSimulation()
time.sleep(0.5)

def check_obstacle_ahead(rx, ry, angle, dist=0.8):
    fx = rx + math.cos(angle) * dist
    fy = ry + math.sin(angle) * dist
    for _, wx, wy in walls:
        if abs(fx - wx) < 0.6 and abs(fy - wy) < 0.6:
            return True
    if abs(fx) > 4.5 or abs(fy) > 3.5:
        return True
    return False

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

waypoints = [
    (1.5, -2.0),
    (1.5, 1.0),
    (-1.0, 1.0),
    (-1.0, -2.0),
    (1.0, -2.0),
]

phase_offsets = {"fl": 0, "fr": math.pi, "rl": math.pi, "rr": 0}

speed = 0.3
wp_idx = 0
t = 0
loop_count = 0
frame_count = 0
orig_pos = sim.getObjectPosition(base, -1)

print(f"Starting patrol: {len(waypoints)} waypoints")
try:
    while True:
        target = waypoints[wp_idx]
        curr = sim.getObjectPosition(base, -1)
        curr_ori = sim.getObjectOrientation(base, -1)
        
        dx = target[0] - curr[0]
        dy = target[1] - curr[1]
        dist = math.sqrt(dx*dx + dy*dy)

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

        if check_obstacle_ahead(curr[0], curr[1], curr_ori[2]):
            print(f"Obstacle ahead! Turning right...")
            curr_ori[2] -= 0.5
            sim.setObjectOrientation(base, -1, curr_ori)
            animate_legs(t, phase_offsets, True)
            update_camera(curr[0], curr[1], curr_ori[2])
            t += 0.1
            frame_count += 1
            if frame_count % 5 == 0:
                trigger_redraw()
            time.sleep(0.1)
            continue

        step = min(speed * 0.1, dist)
        new_x = curr[0] + step * math.cos(curr_ori[2])
        new_y = curr[1] + step * math.sin(curr_ori[2])

        if abs(angle_diff) > 0.1:
            curr_ori[2] += 0.1 * (1 if angle_diff > 0 else -1)

        sim.setObjectPosition(base, -1, [new_x, new_y, orig_pos[2]])
        sim.setObjectOrientation(base, -1, curr_ori)

        animate_legs(t, phase_offsets, True)
        update_camera(curr[0], curr[1], curr_ori[2])

        t += 0.1
        frame_count += 1
        # Trigger viewport redraw every 5 frames (~0.5s)
        if frame_count % 5 == 0:
            trigger_redraw()
        
        if frame_count % 50 == 0:
            print(f"  pos=({new_x:.2f}, {new_y:.2f}) wp={wp_idx} dist={dist:.2f}")
        
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
    print("Done")
