import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('puppy_core_cpp')
    safety_params = os.path.join(pkg_share, 'config', 'safety.yaml')
    features_params = os.path.join(pkg_share, 'config', 'features.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use /clock for C++ puppy_core nodes',
        ),
        Node(
            package='puppy_core_cpp',
            executable='mode_manager',
            name='mode_manager',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='puppy_core_cpp',
            executable='safety_manager',
            name='safety_manager',
            output='screen',
            parameters=[safety_params, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='puppy_core_cpp',
            executable='battery_status_adapter',
            name='battery_status_adapter',
            output='screen',
            parameters=[safety_params, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='puppy_core_cpp',
            executable='robot_state_aggregator',
            name='robot_state_aggregator',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='puppy_core_cpp',
            executable='mission_manager',
            name='mission_manager',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='puppy_core_cpp',
            executable='goal_dispatcher',
            name='goal_dispatcher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='puppy_core_cpp',
            executable='capability_registry',
            name='capability_registry',
            output='screen',
            parameters=[features_params, {'use_sim_time': use_sim_time}],
        ),
    ])

