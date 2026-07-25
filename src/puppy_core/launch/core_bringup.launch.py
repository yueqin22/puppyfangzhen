"""Core bringup: mode manager + safety manager + state aggregator."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('puppy_core')
    safety_params = os.path.join(pkg_share, 'config', 'safety.yaml')
    features_params = os.path.join(pkg_share, 'config', 'features.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use /clock for puppy_core nodes',
        ),
        Node(
            package='puppy_core',
            executable='mode_manager',
            name='mode_manager',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='puppy_core',
            executable='safety_manager',
            name='safety_manager',
            output='screen',
            parameters=[safety_params, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='puppy_core',
            executable='battery_status_adapter',
            name='battery_status_adapter',
            output='screen',
            parameters=[safety_params, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='puppy_core',
            executable='state_aggregator',
            name='robot_state_aggregator',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='puppy_core',
            executable='mission_manager',
            name='mission_manager',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='puppy_core',
            executable='goal_dispatcher',
            name='goal_dispatcher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='puppy_core',
            executable='capability_registry',
            name='capability_registry',
            output='screen',
            parameters=[features_params, {'use_sim_time': use_sim_time}],
        ),
    ])
