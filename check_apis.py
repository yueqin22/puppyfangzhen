#!/usr/bin/env python3
"""Check available sensor/collision APIs in CoppeliaSim ZMQ remote API."""
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

s = RemoteAPIClient().getObject("sim")

apis = [
    "createProximitySensor", "handleProximitySensor", "readProximitySensor",
    "checkCollision", "checkDistance", "getCollisionHash",
    "getVisionSensorDepth", "getVisionSensorFloatParam",
    "raycast", "raycastEx", "checkOctreePointOccupancy",
    "getShapeGeomInfo", "getShapeMesh", "getObjectSize",
    "getObjects", "getObjectsInTree", "getObjectChild",
    "buildMatrix", "getObjectsMatchingRegex",
]
print("=== API availability ===", flush=True)
for name in apis:
    available = hasattr(s, name)
    print(f"  sim.{name}: {'YES' if available else 'NO'}", flush=True)

# Check sim constants for proximity sensor types
print("\n=== Proximity sensor type constants ===", flush=True)
for name in ["proximitysensor_ray", "proximitysensor_pyramid", "proximitysensor_cylinder",
             "handle_scene", "handle_world"]:
    val = getattr(s, name, "NOT_FOUND")
    print(f"  sim.{name}: {val}", flush=True)

# Try listing all objects in scene to find walls with real geometry
print("\n=== Scene objects with geometry ===", flush=True)
count = 0
for obj in s.getObjectsInTree(s.handle_scene):
    try:
        alias = s.getObjectAlias(obj)
        pos = s.getObjectPosition(obj, -1)
        # Try getShapeMesh
        try:
            mesh = s.getShapeMesh(obj)
            print(f"  {alias} (handle={obj}): mesh type={type(mesh)}, len={len(mesh) if mesh else 0}", flush=True)
        except Exception as e:
            pass
        count += 1
        if count > 30:
            break
    except:
        pass
