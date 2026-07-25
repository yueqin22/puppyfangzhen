#!/usr/bin/env python3
"""Reset robot to start position."""
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
s = RemoteAPIClient().getObject("sim")
base = s.getObject("/base_footprint")
s.setObjectPosition(base, -1, [1.0, -2.0, 0.0])
s.setObjectOrientation(base, -1, [0, 0, 0])
pos = s.getObjectPosition(base, -1)
print(f"Robot reset to ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")

# Quick verify checkDistance works from this position
for name in ["wall_south", "wall_north", "wall_divide_2"]:
    try:
        h = s.getObject(f"/{name}")
        result = s.checkDistance(base, h, 10.0)
        if result and result[0]:
            dd = result[1]
            dist = dd[6]
            near_x, near_y = dd[3], dd[4]
            print(f"  {name}: dist={dist:.3f} near=({near_x:.2f},{near_y:.2f})")
        else:
            print(f"  {name}: NOT DETECTED")
    except Exception as e:
        print(f"  {name}: ERROR {e}")
