"""Stop the currently running CoppeliaSim simulation."""
import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

c = RemoteAPIClient()
s = c.getObject('sim')
st = s.getSimulationState()
print(f"State before: {st}")
if st == s.simulation_advancing_running:
    s.stopSimulation()
    time.sleep(3)
    print("Stopped")
print(f"State after: {s.getSimulationState()}")
# Reset robot position
base = s.getObject('/base_footprint')
s.setObjectPosition(base, -1, [1.0, -2.0, 0.0])
s.setObjectOrientation(base, -1, [0, 0, 0])
print("Robot reset to (1, -2, 0)")
