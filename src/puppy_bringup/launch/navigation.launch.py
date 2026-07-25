import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_worlds = get_package_share_directory('puppy_worlds')
    pkg_gait = get_package_share_directory('puppy_gait')
    pkg_nav = get_package_share_directory('puppy_nav')
    pkg_bringup = get_package_share_directory('puppy_bringup')

    # Arguments
    world_arg = DeclareLaunchArgument(
        'world', default_value='small_room',
        description='Gazebo world')
    gui_arg = DeclareLaunchArgument(
        'gui', default_value='false',
        description='Set to "true" to enable Gazebo GUI')
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Set to "false" to disable RViz')
    map_arg = DeclareLaunchArgument(
        'map', default_value=os.path.join(pkg_nav, 'maps', 'small_room.yaml'),
        description='Map yaml file')

    # 1) Gazebo + spawn robot
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_worlds, 'launch', 'simulation.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
        }.items(),
    )

    # 2) Nav2 navigation
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'navigation.launch.py')),
        launch_arguments={'map': LaunchConfiguration('map')}.items(),
    )

    # 4) RViz2
    rviz_config = os.path.join(pkg_bringup, 'rviz', 'navigation.rviz')
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        world_arg,
        gui_arg,
        rviz_arg,
        map_arg,
        simulation,
        TimerAction(period=10.0, actions=[navigation]),
        TimerAction(period=12.0, actions=[rviz2]),
    ])
