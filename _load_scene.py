#!/usr/bin/env python3
"""Load the home_coppelia.ttt scene into CoppeliaSim via ZMQ Remote API."""
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.getObject("sim")

scene_path = "/home/veni/CoppeliaSim/E:\\puppyfangzhen\\src\\puppy_worlds\\worlds\\home_coppelia.ttt"
print(f"Loading scene: {scene_path}")
sim.loadScene(scene_path)
print("Scene loaded.")

# Verify base_footprint exists
try:
    base = sim.getObject("/base_footprint")
    print(f"base_footprint handle: {base} — OK")
except Exception as e:
    print(f"ERROR: base_footprint not found: {e}")

# Reset simulation
sim.stopSimulation()
import time
time.sleep(1)
print("Simulation stopped and ready.")
