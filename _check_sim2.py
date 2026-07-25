#!/usr/bin/env python3
import sys, time
sys.path.insert(0, '/home/veni/CoppeliaSim/programming/zmqRemoteApi/clients/python')
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
client = RemoteAPIClient()
sim = client.getObject('sim')
state = sim.getSimulationState()
stime = sim.getSimulationTime()
state_names = {0:'STOPPED',1:'PAUSED',3:'RUNNING',5:'ADVANCING_PAUSED'}
print(f"State: {state} ({state_names.get(state,'?')})")
print(f"Time: {stime:.2f}s")

try:
    obj = sim.getObject('/base_footprint')
    pos = sim.getObjectPosition(obj, -1)
    print(f"/base_footprint found at ({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f})")
except Exception as e:
    print(f"Error getting /base_footprint: {e}")
    try:
        obj = sim.getObject('/Puppy')
        pos = sim.getObjectPosition(obj, -1)
        print(f"/Puppy found at ({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f})")
    except Exception as e2:
        print(f"Error getting /Puppy: {e2}")
        print("Scene objects (first 20):")
        try:
            objects = sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0)
            for i, h in enumerate(objects[:30]):
                try:
                    name = sim.getObjectAlias(h, 1)
                    print(f"  {h}: {name}")
                except:
                    print(f"  {h}: (error)")
        except Exception as e3:
            print(f"  Error listing objects: {e3}")

client.__del__()
