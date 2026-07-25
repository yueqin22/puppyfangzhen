#!/usr/bin/env python3
"""Test checkDistance during simulation - does it work?"""
import math, time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

s = RemoteAPIClient().getObject("sim")
base = s.getObject("/base_footprint")

# Get wall handles
walls = {}
for name in ["wall_south", "wall_north", "wall_east", "wall_west",
             "wall_divide_1", "wall_divide_2", "sofa", "bed", "dining_table"]:
    try:
        h = s.getObject(f"/{name}")
        walls[name] = h
    except:
        try:
            # Try without leading slash
            h = s.getObject(f"{name}")
            walls[name] = h
        except:
            print(f"  Cannot find {name}")

print(f"Found {len(walls)} objects")
print(f"base handle = {base}")

# Test BEFORE simulation
print("\n--- BEFORE simulation ---")
pos = s.getObjectPosition(base, -1)
print(f"Robot pos: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")
for name, h in list(walls.items())[:3]:
    try:
        result = s.checkDistance(base, h, 10.0)
        print(f"  {name}: result={result}")
    except Exception as e:
        print(f"  {name}: ERROR {e}")

# Start simulation
print("\n--- Starting simulation ---")
s.startSimulation()
time.sleep(1.0)

# Test DURING simulation
print("\n--- DURING simulation ---")
pos = s.getObjectPosition(base, -1)
print(f"Robot pos: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")
ori = s.getObjectOrientation(base, -1)
print(f"Robot ori: ({ori[0]:.3f}, {ori[1]:.3f}, {ori[2]:.3f})")

for name, h in walls.items():
    try:
        result = s.checkDistance(base, h, 10.0)
        if result and result[0]:
            dd = result[1]
            dist = dd[6] if len(dd) > 6 else -1
            near_x, near_y = dd[3], dd[4]
            print(f"  {name}: dist={dist:.3f} near_pt=({near_x:.2f},{near_y:.2f}) "
                  f"full_dd={[round(v,3) for v in dd]}")
        else:
            print(f"  {name}: NOT DETECTED (result={result})")
    except Exception as e:
        print(f"  {name}: ERROR {e}")

# Also check: is base_footprint a shape with geometry?
print(f"\n--- base_footprint info ---")
try:
    alias = s.getObjectAlias(base)
    print(f"  alias: {alias}")
except:
    pass
try:
    # Check if it's a shape
    obj_type = s.getObjectType(base)
    print(f"  type: {obj_type}")
except:
    pass

s.stopSimulation()
time.sleep(0.5)
