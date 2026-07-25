#!/usr/bin/env python3
import sys
sys.path.insert(0, '/home/veni/CoppeliaSim/programming/zmqRemoteApi/clients/python')
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
client = RemoteAPIClient()
sim = client.getObject('sim')
state = sim.getSimulationState()
stime = sim.getSimulationTime()
print(f"State={state} Time={stime:.2f}s")
try:
    obj = sim.getObject('/base_footprint')
    pos = sim.getObjectPosition(obj, -1)
    print(f"base_footprint=({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f})")
except Exception as e:
    print(f"base_footprint error: {e}")
try:
    obj = sim.getObject('/Puppy')
    pos = sim.getObjectPosition(obj, -1)
    print(f"Puppy=({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f})")
except Exception as e:
    print(f"Puppy error: {e}")
client.__del__()
