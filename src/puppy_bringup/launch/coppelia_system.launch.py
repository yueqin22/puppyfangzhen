#!/usr/bin/env python3
"""
Puppy 机械狗 - CoppeliaSim 版完整系统启动文件
不启动 Gazebo，而是连接到已运行的 CoppeliaSim 仿真
启动: odom_tf_publisher + robot_state_publisher + Nav2 + RViz + 功能节点

前提条件:
  1. CoppeliaSim 已启动并加载了 home_coppelia.ttt 场景
  2. 仿真已启动（运行 simple_attach.py 或 reattach_script.py）

用法:
  ros2 launch puppy_bringup coppelia_system.launch.py
  ros2 launch puppy_bringup coppelia_system.launch.py patrol:=true
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_nav = get_package_share_directory('puppy_nav')
    pkg_bringup = get_package_share_directory('puppy_bringup')
    pkg_description = get_package_share_directory('puppy_description')

    # ===== 参数 =====
    map_arg = DeclareLaunchArgument(
        'map', default_value=os.path.join(pkg_nav, 'maps', 'home_map.yaml'),
        description='地图 yaml 文件路径')

    patrol_arg = DeclareLaunchArgument(
        'patrol', default_value='false',
        description='是否自动启动巡逻')

    # ===== 0. CoppeliaSim ZMQ 桥接节点（必须最先启动）=====
    # 通过 ZMQ Remote API 直接与 CoppeliaSim 通信，绕过 simROS2 插件
    # 发布: /odom, /scan, /clock, /tf(odom->base_footprint)
    # 订阅: /cmd_vel
    # use_sim_time=false，因为本节点是时钟源
    coppelia_bridge = Node(
        package='puppy_bringup',
        executable='coppelia_bridge.py',
        name='coppelia_bridge',
        parameters=[{'use_sim_time': False}],
        output='screen',
    )

    # ===== 1. robot_state_publisher（发布 base_footprint->base_link->laser_link TF）=====
    xacro_file = os.path.join(pkg_description, 'urdf', 'puppy.urdf.xacro')
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': True,
            'robot_description': ParameterValue(
                Command(['xacro ', xacro_file]),
                value_type=str),
        }],
        output='screen',
    )

    # ===== 3. Nav2 导航 =====
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'navigation.launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': 'true',
        }.items(),
    )

    # ===== 4. RViz2 =====
    rviz_config = os.path.join(pkg_bringup, 'rviz', 'navigation.rviz')
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ===== 5. 跌倒检测节点 =====
    fall_detection = Node(
        package='puppy_gait',
        executable='fall_detection.py',
        name='fall_detection',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ===== 6. 家庭安防节点 =====
    security_node = Node(
        package='puppy_gait',
        executable='security_node.py',
        name='security_node',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ===== 7. 情绪交互节点 =====
    emotion_interaction = Node(
        package='puppy_gait',
        executable='emotion_interaction.py',
        name='emotion_interaction',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ===== 8. 电池仿真节点 =====
    battery_simulator = Node(
        package='puppy_gait',
        executable='battery_simulator.py',
        name='battery_simulator',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ===== 9. 智能家居家电管理节点 =====
    smart_home_manager = Node(
        package='puppy_gait',
        executable='smart_home_manager.py',
        name='smart_home_manager',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ===== 10. 定时任务调度节点 =====
    task_scheduler = Node(
        package='puppy_gait',
        executable='task_scheduler.py',
        name='task_scheduler',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ===== 11. 紧急联动响应节点 =====
    emergency_response = Node(
        package='puppy_gait',
        executable='emergency_response.py',
        name='emergency_response',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ===== 12. 自动巡逻（可选）=====
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
        map_arg,
        patrol_arg,
        # 0. 立即启动 ZMQ 桥接节点（发布 /odom /scan /clock /tf）
        coppelia_bridge,
        robot_state_publisher,
        # 1. 延迟 3s 启动 Nav2（等待 TF 和 /clock 就绪）
        TimerAction(period=3.0, actions=[navigation]),
        # 2. 延迟 6s 启动 RViz（等待 Nav2 开始配置）
        TimerAction(period=6.0, actions=[rviz2]),
        # 3. 延迟 8s 启动功能节点（等待 Nav2 激活）
        TimerAction(period=8.0, actions=[
            fall_detection,
            security_node,
            emotion_interaction,
            battery_simulator,
            smart_home_manager,
            task_scheduler,
            emergency_response,
        ]),
        # 4. 延迟 15s 启动巡逻（等待 Nav2 完全就绪）
        TimerAction(period=15.0, actions=[patrol]),
    ])
