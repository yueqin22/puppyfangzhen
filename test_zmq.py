#!/usr/bin/env python3
import sys
print("step1: importing", flush=True)
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
print("step2: import done, creating client", flush=True)
client = RemoteAPIClient()
print("step3: client created, getting sim", flush=True)
sim = client.getObject("sim")
print("step4: got sim, calling getSimulationState", flush=True)
st = sim.getSimulationState()
print(f"step5: simulation state = {st}", flush=True)
print("ZMQ OK", flush=True)
