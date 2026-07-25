#!/usr/bin/env python3
"""Get wall geometry: position, size, orientation to build proper collision."""
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import math

s = RemoteAPIClient().getObject("sim")

walls_info = []
for obj in s.getObjectsInTree(s.handle_scene):
    try:
        alias = s.getObjectAlias(obj)
        if any(k in alias for k in ["wall_", "sofa", "bed", "dining_table"]):
            pos = s.getObjectPosition(obj, -1)
            ori = s.getObjectOrientation(obj, -1)
            # Try getObjectSize - returns [x,y,z] size of bounding box
            try:
                size = s.getObjectSize(obj)
            except Exception:
                size = [0,0,0]
            # Also try getShapeGeometry if available
            yaw = ori[2]
            walls_info.append((alias, pos[0], pos[1], pos[2], size[0], size[1], size[2], yaw))
    except:
        pass

print(f"Found {len(walls_info)} obstacles:")
for w in walls_info:
    alias, x, y, z, sx, sy, sz, yaw = w
    # Compute wall endpoints (long axis)
    # If size x > size y, wall is along X; else along Y
    if sx >= sy:
        half = sx / 2.0
        x1 = x - half * math.cos(yaw)
        y1 = y - half * math.sin(yaw)
        x2 = x + half * math.cos(yaw)
        y2 = y + half * math.sin(yaw)
    else:
        half = sy / 2.0
        x1 = x - half * math.sin(yaw)
        y1 = y + half * math.cos(yaw)
        x2 = x + half * math.sin(yaw)
        y2 = y - half * math.cos(yaw)
    print(f"  {alias}: pos=({x:.2f},{y:.2f},{z:.2f}) size=({sx:.2f},{sy:.2f},{sz:.2f}) yaw={yaw:.2f}")
    print(f"    endpoints: ({x1:.2f},{y1:.2f}) -> ({x2:.2f},{y2:.2f})")
