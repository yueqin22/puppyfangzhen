#!/usr/bin/env python3
"""Get wall mesh geometry + test checkDistance + test createProximitySensor."""
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import math

s = RemoteAPIClient().getObject("sim")

# 1. Get mesh data for walls
print("=== Wall mesh data ===", flush=True)
obstacles = []
for obj in s.getObjectsInTree(s.handle_scene):
    try:
        alias = s.getObjectAlias(obj)
        if any(k in alias for k in ["wall_", "sofa", "bed", "dining_table"]):
            pos = s.getObjectPosition(obj, -1)
            ori = s.getObjectOrientation(obj, -1)
            mesh = s.getShapeMesh(obj)
            # mesh is (vertices, indices, normals) typically
            if isinstance(mesh, (list, tuple)) and len(mesh) >= 2:
                verts = mesh[0]
                indices = mesh[1]
                n_verts = len(verts) // 3
                n_tris = len(indices) // 3
                # Get bounding box from vertices
                xs = verts[0::3]; ys = verts[1::3]; zs = verts[2::3]
                print(f"  {alias}: pos=({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f}) "
                      f"verts={n_verts} tris={n_tris} "
                      f"bbox x=[{min(xs):.2f},{max(xs):.2f}] y=[{min(ys):.2f},{max(ys):.2f}] z=[{min(zs):.2f},{max(zs):.2f}]",
                      flush=True)
                obstacles.append((alias, obj, pos, ori, verts, indices))
    except Exception as e:
        pass

# 2. Test sim.checkDistance
print("\n=== Test checkDistance ===", flush=True)
base = s.getObject("/base_footprint")
for alias, obj, pos, ori, _, _ in obstacles[:3]:
    try:
        result = s.checkDistance(base, obj, 10.0)
        print(f"  to {alias}: result={result}", flush=True)
    except Exception as e:
        print(f"  to {alias}: ERROR {e}", flush=True)

# 3. Test createProximitySensor with different type values
print("\n=== Test createProximitySensor ===", flush=True)
for stype in [48, 49, 50, 0, 1, 2]:
    try:
        # Try minimal params
        ps = s.createProximitySensor(stype, 0, [0,0,0,0], [0.01, 5.0, 0.1, 0.1, 0.1, 0.1, 0.5,0.5,0.5, 0,0])
        print(f"  type={stype}: SUCCESS handle={ps}", flush=True)
        s.removeObject(ps)
        break
    except Exception as e:
        print(f"  type={stype}: {e}", flush=True)

# 4. Test getVisionSensorDepth
print("\n=== Test getVisionSensorDepth ===", flush=True)
try:
    vs = s.getObject("/patrol_camera")
    if vs:
        depth = s.getVisionSensorDepth(vs)
        print(f"  depth type={type(depth)} len={len(depth) if depth else 0}", flush=True)
except Exception as e:
    print(f"  ERROR: {e}", flush=True)
