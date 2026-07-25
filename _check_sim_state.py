#!/usr/bin/env python3
"""Check CoppeliaSim simulation state and reset if needed."""
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.getObject('sim')

state = sim.getSimulationState()
state_names = {
    0: 'stopped',
    16: 'paused',
    17: 'advancing_running',
    18: 'advancing_firstafterstop',
    19: 'advancing_lastbeforestop',
}
print(f'Simulation state: {state} ({state_names.get(state, "unknown")})')

# Check if base_footprint exists
try:
    base = sim.getObject('/base_footprint')
    pos = sim.getObjectPosition(base, -1)
    print(f'Base position: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})')
except:
    print('ERROR: base_footprint not found - need to rebuild scene')

# Stop simulation if running, then reset
if state == sim.simulation_advancing_running:
    print('Stopping simulation...')
    sim.stopSimulation()
    import time
    time.sleep(2)

# Reset robot to start position
sim.setObjectPosition(base, -1, [1.0, -2.0, 0.0])
sim.setObjectOrientation(base, -1, [0, 0, 0])
print('Robot reset to (1.0, -2.0, 0.0)')
print('Ready for new simulation run')
