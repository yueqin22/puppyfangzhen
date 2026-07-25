import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('puppypi_adapter')
    default_params_file = os.path.join(pkg_share, 'config', 'adapter_params.yaml')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='ROS2 parameter file for PuppyPi adapter nodes',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use /clock for adapter node timing',
        ),
        Node(
            package='puppypi_adapter',
            executable='motion_adapter',
            name='motion_adapter',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='puppypi_adapter',
            executable='status_adapter',
            name='status_adapter',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='puppypi_adapter',
            executable='mode_adapter',
            name='mode_adapter',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),
    ])
