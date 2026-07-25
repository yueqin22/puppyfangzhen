import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('puppy_nav')
    params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    default_map = os.path.join(pkg_share, 'maps', 'home_map.yaml')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Map yaml file path')

    nav2_params = DeclareLaunchArgument(
        'params_file',
        default_value=params_file,
        description='Nav2 params file')

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    # Map server
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'yaml_filename': LaunchConfiguration('map'),
                     'use_sim_time': True}],
        output='screen',
    )

    # AMCL
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        parameters=[params_file, {'use_sim_time': True}],
        output='screen',
    )

    # Controller server
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        parameters=[params_file, {'use_sim_time': True}],
        output='screen',
    )

    # Planner server
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        parameters=[params_file, {'use_sim_time': True}],
        output='screen',
    )

    # Behavior server
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        parameters=[params_file, {'use_sim_time': True}],
        output='screen',
    )

    # BT Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        parameters=[params_file, {'use_sim_time': True}],
        output='screen',
    )

    # Waypoint follower
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        parameters=[params_file, {'use_sim_time': True}],
        output='screen',
    )

    # Smoother server
    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        parameters=[params_file, {'use_sim_time': True}],
        output='screen',
    )

    # Lifecycle manager - 包含所有节点，map_server 放第一位
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': True,
                     'autostart': True,
                     'bond_timeout': 10.0,
                     'lifecycle_configure_delay': 1.0,
                     'node_names': ['map_server',
                                    'amcl',
                                    'controller_server',
                                    'planner_server',
                                    'behavior_server',
                                    'bt_navigator',
                                    'waypoint_follower',
                                    'smoother_server']}],
    )

    return LaunchDescription([
        map_arg,
        nav2_params,
        sim_time_arg,
        map_server,
        amcl,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        smoother_server,
        lifecycle_manager,
    ])
