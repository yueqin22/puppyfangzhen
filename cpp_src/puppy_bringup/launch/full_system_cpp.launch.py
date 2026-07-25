import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    include_adapter = LaunchConfiguration('include_adapter')
    include_gait = LaunchConfiguration('include_gait')
    include_patrol_trigger = LaunchConfiguration('patrol')

    core_launch = os.path.join(
        get_package_share_directory('puppy_core_cpp'),
        'launch',
        'core_bringup_cpp.launch.py',
    )
    adapter_launch = os.path.join(
        get_package_share_directory('puppypi_adapter_cpp'),
        'launch',
        'adapter_bringup_cpp.launch.py',
    )
    gait_launch = os.path.join(
        get_package_share_directory('puppy_gait_cpp'),
        'launch',
        'gait_nodes_cpp.launch.py',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use /clock for C++ bringup nodes',
        ),
        DeclareLaunchArgument(
            'include_adapter',
            default_value='true',
            description='Start C++ PuppyPi adapter nodes',
        ),
        DeclareLaunchArgument(
            'include_gait',
            default_value='true',
            description='Start C++ gait/application nodes',
        ),
        DeclareLaunchArgument(
            'patrol',
            default_value='false',
            description='Send one patrol command through puppy_nav_cpp',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(core_launch),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(adapter_launch),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
            condition=IfCondition(include_adapter),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gait_launch),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
            condition=IfCondition(include_gait),
        ),
        Node(
            package='puppy_nav_cpp',
            executable='patrol',
            name='patrol_trigger',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(include_patrol_trigger),
        ),
    ])
