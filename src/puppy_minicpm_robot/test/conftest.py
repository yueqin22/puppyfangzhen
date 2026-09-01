# -*- coding: utf-8 -*-
"""Shared pytest fixtures for the puppy_minicpm_robot test suite.

The nodes under test construct a real rclpy node whenever rclpy is importable
(see the `HAS_RCLPY` guards in each node module). That is the case under
`colcon test` inside a sourced ROS workspace -- exactly the environment these
tests exist to protect -- but creating a node requires an initialised context.

Without the fixture below every node-constructing test errors out during setup
as soon as rclpy is importable ("Context of the node is not valid"), so the
suite only ever passed on hosts without ROS installed. Initialising the default
context once per session makes the suite run identically in both environments.
"""

import pytest

try:
    import rclpy
    HAS_RCLPY = True
except ImportError:
    HAS_RCLPY = False


@pytest.fixture(scope="session", autouse=True)
def ros_context():
    """Initialise (and tear down) the default rclpy context for the session."""
    if not HAS_RCLPY:
        yield None
        return

    initialised_here = False
    if not rclpy.ok():
        rclpy.init(args=[])
        initialised_here = True

    try:
        yield rclpy.get_default_context()
    finally:
        if initialised_here and rclpy.ok():
            rclpy.shutdown()
