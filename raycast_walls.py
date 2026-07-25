#!/usr/bin/env python3
"""Use raycasting to find actual wall extents."""
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import math

s = RemoteAPIClient().getObject("sim")

def ray(x, y, z, dx, dy, dz, maxDist=10.0):
    """Cast a ray from (x,y,z) in direction (dx,dy,dz)."""
    # Build 4x4 matrix: columns are X,Y,Z axes + position
    # Z axis = ray direction (normalized)
    d = math.sqrt(dx*dx+dy*dy+dz*dz)
    fx, fy, fz = dx/d, dy/d, dz/d
    # pick an up vector not parallel to forward
    ux, uy, uz = 0, 0, 1
    if abs(fz) > 0.9:
        ux, uy, uz = 0, 1, 0
    # right = forward x up
    rx = fy*uz - fz*uy
    ry = fz*ux - fx*uz
    rz = fx*uy - fy*ux
    r = math.sqrt(rx*rx+ry*ry+rz*rz)
    rx, ry, rz = rx/r, ry/r, rz/r
    # up = right x forward
    upx = ry*fz - rz*fy
    upy = rz*fx - rx*fz
    upz = rx*fy - ry*fx
    # matrix 3x4 row-major: [right, up, forward, pos] -> CoppeliaSim uses [rx ux fx px; ry uy fy py; rz uz fz pz]
    m = [rx, upx, fx, x,
         ry, upy, fy, y,
         rz, upz, fz, z]
    try:
        ret = s.raycast(m, maxDist, 0.0, -1)
        if ret:
            return ret
        return None
    except Exception as e:
        return f"ERR: {e}"

# Test: ray along divide walls to find their Y extent
print("=== Testing wall_divide_2 (x=3) Y extent ===", flush=True)
for y in [-3.5, -3, -2, -1, 0, 1, 2, 3, 3.5]:
    r = ray(2.5, y, 0.3, 1, 0, 0, 2.0)  # ray +X from x=2.5
    print(f"  y={y}: {r}", flush=True)

print("\n=== Testing wall_divide_1 (x=-3) Y extent ===", flush=True)
for y in [-3.5, -3, -2, -1, 0, 1, 2, 3, 3.5]:
    r = ray(-2.5, y, 0.3, -1, 0, 0, 2.0)  # ray -X from x=-2.5
    print(f"  y={y}: {r}", flush=True)

print("\n=== Testing south/north walls ===", flush=True)
for x in [-4, -2, 0, 2, 4]:
    r = ray(x, -3, 0.3, 0, -1, 0, 2.0)  # ray -Y
    print(f"  south x={x}: {r}", flush=True)
    r = ray(x, 3, 0.3, 0, 1, 0, 2.0)  # ray +Y
    print(f"  north x={x}: {r}", flush=True)
