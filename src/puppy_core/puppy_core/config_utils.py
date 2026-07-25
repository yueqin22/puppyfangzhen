"""Helpers for loading shared runtime configuration."""
import os
from typing import Dict, List, Optional

import yaml
from ament_index_python.packages import get_package_share_directory


def _load_yaml(filename: str) -> dict:
    pkg_share = get_package_share_directory('puppy_core')
    path = os.path.join(pkg_share, 'config', filename)
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f) or {}


def load_navigation_targets() -> Dict[str, dict]:
    """Load named navigation targets from shared config."""
    data = _load_yaml('navigation_targets.yaml')
    return data.get('targets', {})


def get_named_target(name: str, default: Optional[dict] = None) -> Optional[dict]:
    """Return a named target dict with x/y/yaw, or default if missing."""
    targets = load_navigation_targets()
    return targets.get(name, default)


def get_dock_target(default: Optional[dict] = None) -> Optional[dict]:
    """Return the configured dock target."""
    return get_named_target('dock', default)


def load_patrol_routes() -> Dict[str, List[dict]]:
    """Load named patrol routes from shared config."""
    data = _load_yaml('patrol_routes.yaml')
    return data.get('routes', {})


def get_patrol_route(name: str = 'default', default: Optional[List[dict]] = None) -> List[dict]:
    """Return a patrol route as a list of named waypoint dicts."""
    routes = load_patrol_routes()
    route = routes.get(name)
    if route is None:
        return list(default or [])
    return list(route)
