import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_worlds = get_package_share_directory('puppy_worlds')
    pkg_gait = get_package_share_directory('puppy_gait')
    pkg_slam = get_package_share_directory('puppy_slam')
    pkg_bringup = get_package_share_directory('puppy_bringup')

    # Arguments
    world_arg = DeclareLaunchArgument(
        'world', default_value='small_room',
        description='Gazebo world (small_room, office)')
    gui_arg = DeclareLaunchArgument(
        'gui', default_value='false',
        description='Set to "false" to disable gzclient')
    server_arg = DeclareLaunchArgument(
        'server', default_value='true',
        description='Set to "false" to disable gzserver')
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Set to "false" to disable RViz')
    teleop_arg = DeclareLaunchArgument(
        'teleop', default_value='false',
        description='Set to "true" to enable teleop_twist_keyboard in xterm')

    # 1) Gazebo + spawn robot
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_worlds, 'launch', 'simulation.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
            'server': LaunchConfiguration('server'),
        }.items(),
    )

    # 2) Gait controller
    gait_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gait, 'launch', 'gait_controller.launch.py')),
    )

    # 3) SLAM (slam_toolbox)
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam, 'launch', 'slam_toolbox.launch.py')),
    )

    # 4) Teleop keyboard
    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        prefix='xterm -e',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('teleop')),
    )

    # 5) RViz2
    rviz_config = os.path.join(pkg_bringup, 'rviz', 'mapping.rviz')
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
        server_arg,
        rviz_arg,
        teleop_arg,
        simulation,
        gait_controller,
        slam,
        teleop,
        rviz2,
    ])
