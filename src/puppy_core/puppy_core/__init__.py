"""Puppy Core: platform-agnostic robot capability layer.

Contains:
  - mode_manager: Robot mode state machine (IDLE/READY/NAV/PATROL/DOCKING/SAFE_STOP/FAULT)
  - safety_manager: Safety monitoring and emergency response
  - mission_manager: High-level task orchestration
  - robot_state_aggregator: Unified robot state from multiple sources
  - capability_registry: Feature flags for graceful degradation
  - goal_dispatcher: Navigation goal routing
"""
__version__ = '1.0.0'
