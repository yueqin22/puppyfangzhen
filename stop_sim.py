#!/usr/bin/env python3
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
s = RemoteAPIClient().getObject("sim")
s.stopSimulation()
time.sleep(1)
print("sim state:", s.getSimulationState())
