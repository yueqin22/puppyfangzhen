#!/usr/bin/env python3
"""Diagnostic script for vision sensor rendering in software rendering mode."""
import math, sys
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from PIL import Image
import io

client = RemoteAPIClient()
sim = client.getObject("sim")

# Check simulation state
sim_state = sim.getSimulationState()
print(f"Simulation state: {sim_state} (0=stopped, 16=running)")

# Get robot position
base = sim.getObject("/base_footprint")
rpos = sim.getObjectPosition(base, -1)
print(f"Robot position: {rpos}")

# Remove old vision sensor
try:
    old_vs = sim.getObject("/patrol_camera")
    sim.removeObject(old_vs)
    print(f"Removed old vision sensor: {old_vs}")
except:
    pass

# Create vision sensor
angle_rad = 60.0 * math.pi / 180.0
vs = sim.createVisionSensor(
    3,  # bit0=explicit, bit1=perspective
    [640, 480, 0, 0],
    [0.01, 50.0, angle_rad, 0.1, 0, 0, 0.5, 0.5, 0.5, 0, 0]
)
sim.setObjectAlias(vs, "patrol_camera")
print(f"Created vision sensor: handle={vs}")

# Position camera above the scene looking down
sim.setObjectPosition(vs, -1, [0, 0, 8.0])
sim.setObjectOrientation(vs, -1, [math.pi, 0, 0])  # Look straight down
print("Set vision sensor position: (0, 0, 8) looking down")

# Start simulation if not running
if sim_state == 0:
    print("Starting simulation...")
    sim.startSimulation()
    import time
    time.sleep(1)

# Handle vision sensor
print("Calling handleVisionSensor...")
ret = sim.handleVisionSensor(vs)
print(f"handleVisionSensor returned: {ret}")

# Get image - try different APIs
print("Trying getVisionSensorImg...")
try:
    result = sim.getVisionSensorImg(vs)
    print(f"getVisionSensorImg returned: type={type(result)}, len={len(result)}")
    if isinstance(result, (list, tuple)) and len(result) >= 2:
        img_data = result[0]
        res = result[1]
        print(f"  img_data type: {type(img_data)}, len: {len(img_data) if img_data else 0}")
        print(f"  resolution: {res}")
        if img_data:
            # Check data content
            sample = img_data[:30] if isinstance(img_data, (bytes, bytearray)) else list(img_data[:30])
            print(f"  first 30 bytes: {sample}")
            nonzero = sum(1 for b in img_data[:1000] if b != 0)
            print(f"  nonzero bytes (first 1000): {nonzero}")
            
            # Try creating image
            if res and len(res) >= 2:
                w, h = res[0], res[1]
                expected_rgb = w * h * 3
                expected_rgba = w * h * 4
                expected_gray = w * h
                print(f"  Expected data lengths - RGB:{expected_rgb}, RGBA:{expected_rgba}, Gray:{expected_gray}")
                print(f"  Actual data length: {len(img_data)}")
                
                # Try RGB
                if len(img_data) == expected_rgb:
                    img = Image.frombytes("RGB", (w, h), img_data)
                    img = img.transpose(Image.FLIP_TOP_BOTTOM)
                    img.save("/tmp/diag_rgb.jpg", format="JPEG", quality=80)
                    print(f"  Saved RGB image: /tmp/diag_rgb.jpg ({len(open('/tmp/diag_rgb.jpg','rb').read())} bytes)")
                elif len(img_data) == expected_rgba:
                    img = Image.frombytes("RGBA", (w, h), img_data)
                    img = img.convert("RGB").transpose(Image.FLIP_TOP_BOTTOM)
                    img.save("/tmp/diag_rgba.jpg", format="JPEG", quality=80)
                    print(f"  Saved RGBA image: /tmp/diag_rgba.jpg")
                elif len(img_data) == expected_gray:
                    img = Image.frombytes("L", (w, h), img_data)
                    img = img.transpose(Image.FLIP_TOP_BOTTOM)
                    img.save("/tmp/diag_gray.jpg", format="JPEG", quality=80)
                    print(f"  Saved grayscale image: /tmp/diag_gray.jpg")
                else:
                    print(f"  Data length doesn't match any expected format!")
                    # Try saving raw data
                    with open("/tmp/diag_raw.dat", "wb") as f:
                        f.write(img_data if isinstance(img_data, (bytes, bytearray)) else bytes(img_data))
                    print(f"  Saved raw data: /tmp/diag_raw.dat")
except Exception as e:
    print(f"getVisionSensorImg error: {e}")

# Also try getVisionSensorImage (legacy API)
print("\nTrying getVisionSensorImage (legacy)...")
try:
    result2 = sim.getVisionSensorImage(vs)
    print(f"getVisionSensorImage returned: type={type(result2)}, len={len(result2)}")
    if isinstance(result2, (list, tuple)) and len(result2) >= 2:
        img_data2 = result2[0]
        res2 = result2[1]
        print(f"  img_data type: {type(img_data2)}, len: {len(img_data2) if img_data2 else 0}")
        print(f"  resolution: {res2}")
        if img_data2:
            nonzero2 = sum(1 for b in img_data2[:1000] if b != 0)
            print(f"  nonzero bytes (first 1000): {nonzero2}")
except Exception as e:
    print(f"getVisionSensorImage error: {e}")

# Try different camera position - looking at robot from angle
print("\n--- Test 2: Camera looking at robot from angle ---")
cam_x = rpos[0] - 4.0
cam_y = rpos[1]
cam_z = 3.5
sim.setObjectPosition(vs, -1, [cam_x, cam_y, cam_z])
dx = rpos[0] - cam_x
dy = rpos[1] - cam_y
dz = rpos[2] - cam_z
yaw = math.atan2(dy, dx)
pitch = math.atan2(-dz, math.sqrt(dx*dx + dy*dy))
sim.setObjectOrientation(vs, -1, [pitch, 0, yaw])
print(f"Camera at ({cam_x:.1f}, {cam_y:.1f}, {cam_z:.1f}) looking at robot")

sim.handleVisionSensor(vs)
try:
    result3 = sim.getVisionSensorImg(vs)
    if isinstance(result3, (list, tuple)) and len(result3) >= 2:
        img_data3 = result3[0]
        res3 = result3[1]
        if img_data3 and res3 and len(res3) >= 2:
            w, h = res3[0], res3[1]
            if len(img_data3) == w * h * 3:
                img = Image.frombytes("RGB", (w, h), img_data3)
                img = img.transpose(Image.FLIP_TOP_BOTTOM)
                img.save("/tmp/diag_angle.jpg", format="JPEG", quality=80)
                print(f"Saved angle view: /tmp/diag_angle.jpg")
                nonzero3 = sum(1 for b in img_data3 if b != 0)
                print(f"Total nonzero bytes: {nonzero3}/{len(img_data3)}")
except Exception as e:
    print(f"Error: {e}")

# Cleanup
print("\nCleanup...")
try:
    sim.removeObject(vs)
except:
    pass
if sim_state == 0:
    sim.stopSimulation()
print("Done")
