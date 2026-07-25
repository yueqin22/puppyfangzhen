"""NavCore: Platform-agnostic navigation kernel.

Re-exports the core navigation algorithms so they can be shared between
the CoppeliaSim standalone runtime and the ROS2 runtime. This is the
single source of truth for mapping, planning, and exploration logic.
"""
from nav_core.mapping import OccupancyGrid, GRID_RESOLUTION, GRID_W, GRID_H, ORIGIN_X, ORIGIN_Y
from nav_core.costmap import Costmap, COST_LETHAL, COST_INSCRIBED
from nav_core.planners import AStarPlanner, DWAPlanner
from nav_core.exploration import FrontierManager
from nav_core.runtime import NavStateMachine, NavState, NavigationRuntimeState
from nav_core.session import SessionStore
from nav_core.metrics import TelemetryCollector
from nav_core.recovery import RecoveryManager, RecoveryContext, RecoveryResult

__version__ = "5.0.0"
__all__ = [
    'OccupancyGrid', 'Costmap', 'AStarPlanner', 'DWAPlanner',
    'FrontierManager', 'NavStateMachine', 'NavState', 'NavigationRuntimeState',
    'SessionStore', 'TelemetryCollector',
    'RecoveryManager', 'RecoveryContext', 'RecoveryResult',
    'GRID_RESOLUTION', 'GRID_W', 'GRID_H', 'ORIGIN_X', 'ORIGIN_Y',
    'COST_LETHAL', 'COST_INSCRIBED',
]
