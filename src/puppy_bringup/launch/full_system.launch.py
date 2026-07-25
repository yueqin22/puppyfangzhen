#!/usr/bin/env python3
"""
Puppy 机械狗 - 完整系统启动文件
启动 Gazebo 仿真 + Nav2 导航 + RViz + 控制面(puppy_core) + 适配层(adapter) + 功能节点

P0-4: 收敛为权威 bringup 路径，包含完整的 4 层架构：
  1. 仿真层（Gazebo）
  2. 导航层（Nav2）
  3. 适配层（puppypi_adapter，use_sim=true）
  4. 控制面（puppy_core: mode/safety/mission/aggregator/dispatcher）
  5. 功能层（fall_detection/security/battery/emotion）
  6. 可选巡逻

用法:
  ros2 launch puppy_bringup full_system.launch.py
  ros2 launch puppy_bringup full_system.launch.py patrol:=true
  ros2 launch puppy_bringup full_system.launch.py gui:=true
"""
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
    pkg_nav = get_package_share_directory('puppy_nav')
    pkg_bringup = get_package_share_directory('puppy_bringup')
    pkg_core = get_package_share_directory('puppy_core')
    pkg_adapter = get_package_share_directory('puppypi_adapter')

    # ===== 参数 =====
    world_arg = DeclareLaunchArgument(
        'world', default_value='home',
        description='Gazebo 世界文件名')

    map_arg = DeclareLaunchArgument(
        'map', default_value=os.path.join(pkg_nav, 'maps', 'home_map.yaml'),
        description='地图 yaml 文件路径')

    gui_arg = DeclareLaunchArgument(
        'gui', default_value='false',
        description='是否启动 Gazebo GUI')

    patrol_arg = DeclareLaunchArgument(
        'patrol', default_value='false',
        description='是否自动启动巡逻')

    # ===== 1. Gazebo 仿真 =====
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_worlds, 'launch', 'simulation.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'gui': LaunchConfiguration('gui'),
            'use_sim_time': 'true',
        }.items(),
    )

    # ===== 2. Nav2 导航 =====
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'navigation.launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': 'true',
        }.items(),
    )

    # P0-4: 3. 控制面（puppy_core: mode/safety/mission/aggregator/dispatcher）
    core_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_core, 'launch', 'core_bringup.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    # P0-4: 4. 适配层（puppypi_adapter，use_sim=true 用于 Gazebo 仿真）
    adapter_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_adapter, 'launch', 'adapter_bringup.launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': os.path.join(pkg_adapter, 'config', 'full_system_adapter_params.yaml'),
        }.items(),
    )

    # ===== 5. RViz2 =====
    rviz_config = os.path.join(pkg_bringup, 'rviz', 'navigation.rviz')
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ===== 6. 跌倒检测节点 =====
    fall_params = os.path.join(pkg_core, 'config', 'features.yaml')
    fall_detection = Node(
        package='puppy_gait',
        executable='fall_detection.py',
        name='fall_detection',
        parameters=[fall_params, {'use_sim_time': True}],
        output='screen',
    )

    # ===== 7. 家庭安防节点 =====
    security_node = Node(
        package='puppy_gait',
        executable='security_node.py',
        name='security_node',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ===== 8. 情绪交互节点 =====
    emotion_interaction = Node(
        package='puppy_gait',
        executable='emotion_interaction.py',
        name='emotion_interaction',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ===== 9. 电池仿真节点 =====
    battery_simulator = Node(
        package='puppy_gait',
        executable='battery_simulator.py',
        name='battery_simulator',
        parameters=[{
            'use_sim_time': True,
            'publish_semantic_battery': False,
        }],
        output='screen',
    )

    # ===== 10. 自动巡逻（可选）=====
    patrol = Node(
        package='puppy_nav',
        executable='patrol.py',
        name='patrol_node',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('patrol')),
        output='screen',
    )

    return LaunchDescription([
        # 参数
        world_arg,
        map_arg,
        gui_arg,
        patrol_arg,
        # 1. 立即启动 Gazebo 仿真（需要 ~15s 加载世界+生成机器人）
        simulation,
        # 2. 延迟 15s 启动 Nav2（等待机器人生成、TF/scan 就绪）
        TimerAction(period=15.0, actions=[navigation]),
        # 3. 延迟 18s 启动 RViz（等待 Nav2 开始配置）
        TimerAction(period=18.0, actions=[rviz2]),
        # P0-4: 4. 延迟 20s 启动控制面+适配层（等待 Nav2 激活）
        TimerAction(period=20.0, actions=[
            core_bringup,
            adapter_bringup,
            fall_detection,
            security_node,
            emotion_interaction,
            battery_simulator,
        ]),
        # 5. 延迟 40s 启动巡逻（等待 Nav2 完全就绪）
        TimerAction(period=40.0, actions=[patrol]),
    ])
