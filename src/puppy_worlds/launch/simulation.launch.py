import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_worlds = get_package_share_directory('puppy_worlds')
    pkg_description = get_package_share_directory('puppy_description')

    world_arg = DeclareLaunchArgument(
        'world', default_value='home',
        description='Gazebo world name')

    gui_arg = DeclareLaunchArgument(
        'gui', default_value='false',
        description='Set to false to disable gzclient GUI')

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true')

    planar_arg = DeclareLaunchArgument(
        'use_planar_move', default_value='true',
        description='Use planar navigation model instead of joint dynamics')

    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'),
                        'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': [pkg_worlds, '/worlds/', LaunchConfiguration('world'), '.world'],
            'gui': LaunchConfiguration('gui'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    # Robot URDF
    urdf_file = os.path.join(pkg_description, 'urdf', 'puppy.urdf.xacro')
    robot_description = Command([
        'xacro ', urdf_file,
        ' use_planar_move:=', LaunchConfiguration('use_planar_move')])

    # Spawn robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_puppy',
        arguments=['-entity', 'puppy',
                   '-topic', '/robot_description',
                   '-x', '1.0', '-y', '-2.0', '-z', '0.30',
                   '-timeout', '120'],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # Robot state publisher with use_sim_time
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(robot_description, value_type=str),
                     'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        world_arg,
        gui_arg,
        sim_time_arg,
        planar_arg,
        gazebo,
        robot_state_publisher,
        spawn_entity,
    ])
