#!/usr/bin/env python3
import sys, time
sys.path.insert(0, '/home/veni/CoppeliaSim/programming/zmqRemoteApi/clients/python')
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

print("Connecting to CoppeliaSim...")
client = RemoteAPIClient()
sim = client.getObject('sim')
state = sim.getSimulationState()
st = sim.getSimulationTime()
print(f"State: {state}, Time: {st:.2f}s")

if state != sim.simulation_advancing_running:
    print("Simulation NOT running! Starting...")
    sim.startSimulation()
    time.sleep(2)
    state = sim.getSimulationState()
    st = sim.getSimulationTime()
    print(f"After start - State: {state}, Time: {st:.2f}s")
else:
    print("Simulation is running.")

# Check time advancement
for i in range(5):
    time.sleep(1)
    st = sim.getSimulationTime()
    print(f"  t+{i+1}s: sim_time={st:.2f}s")

# Check objects
try:
    obj = sim.getObject('/base_footprint')
    pos = sim.getObjectPosition(obj, -1)
    print(f"/base_footprint at ({pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f})")
except Exception as e:
    print(f"/base_footprint error: {e}")

client.__del__()
