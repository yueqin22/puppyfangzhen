#!/usr/bin/env python3
import sys, time
sys.path.insert(0, '/home/veni/CoppeliaSim/programming/zmqRemoteApi/clients/python')
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
client = RemoteAPIClient()
sim = client.getObject('sim')
state = sim.getSimulationState()
stime = sim.getSimulationTime()
state_names = {
    0: 'STOPPED',
    1: 'PAUSED',
    3: 'ADVANCING_RUNNING',
    5: 'ADVANCING_PAUSED',
}
print(f"Simulation state: {state} ({state_names.get(state, 'UNKNOWN')})")
print(f"Simulation time: {stime:.1f}s")

# Get robot handle
try:
    robot = sim.getObject('/Puppy')
    pos = sim.getObjectPosition(robot, -1)
    print(f"Robot position: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")
except Exception as e:
    print(f"Error getting robot: {e}")

client.__del__()
