#!/usr/bin/env python3
"""
Entry Point 1: mock + core
Hardware-free system bringup for unit/integration/contract testing.

Launches:
  1. puppypi_mock (fake motion, fake status, fake sensors)
  2. puppy_core (mode_manager, safety_manager, state_aggregator, mission_manager, goal_dispatcher)
  3. puppypi_adapter (motion/status/mode adapters in mock mode)

Usage:
  ros2 launch puppy_bringup mock_system.launch.py
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_mock = get_package_share_directory('puppypi_mock')
    pkg_core = get_package_share_directory('puppy_core')
    pkg_adapter = get_package_share_directory('puppypi_adapter')

    mock_params = os.path.join(pkg_adapter, 'config', 'mock_adapter_params.yaml')

    mock_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_mock, 'launch', 'mock_bringup.launch.py')
        )
    )

    core_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_core, 'launch', 'core_bringup.launch.py')
        ),
        launch_arguments={'use_sim_time': 'false'}.items(),
    )

    adapter_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_adapter, 'launch', 'adapter_bringup.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': mock_params,
        }.items(),
    )

    return LaunchDescription([
        mock_bringup,
        core_bringup,
        adapter_bringup,
    ])
