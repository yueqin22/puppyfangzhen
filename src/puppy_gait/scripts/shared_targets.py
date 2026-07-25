"""Shared navigation and patrol config for simulation-side helper scripts."""
import os

import yaml
from ament_index_python.packages import get_package_share_directory


def _load_yaml(filename):
    pkg_share = get_package_share_directory('puppy_core')
    path = os.path.join(pkg_share, 'config', filename)
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f) or {}


def load_navigation_targets():
    """Load shared named navigation targets from puppy_core config."""
    data = _load_yaml('navigation_targets.yaml')
    return data.get('targets', {})


def get_named_target(name, default=None):
    """Get a named target dict with x/y/yaw."""
    return load_navigation_targets().get(name, default)


def load_patrol_routes():
    """Load shared patrol routes from puppy_core config."""
    data = _load_yaml('patrol_routes.yaml')
    return data.get('routes', {})


def get_patrol_route(name='default', default=None):
    """Get a named patrol route."""
    route = load_patrol_routes().get(name)
    if route is None:
        return list(default or [])
    return list(route)
