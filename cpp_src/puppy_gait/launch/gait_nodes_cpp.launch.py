from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    common = [{'use_sim_time': use_sim_time}]
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use /clock for C++ puppy_gait nodes',
        ),
        Node(
            package='puppy_gait_cpp',
            executable='fall_detection',
            name='fall_detection',
            output='screen',
            parameters=common,
        ),
        Node(
            package='puppy_gait_cpp',
            executable='security_node',
            name='security_node',
            output='screen',
            parameters=common,
        ),
        Node(
            package='puppy_gait_cpp',
            executable='emotion_interaction',
            name='emotion_interaction',
            output='screen',
            parameters=common,
        ),
        Node(
            package='puppy_gait_cpp',
            executable='battery_simulator',
            name='battery_simulator',
            output='screen',
            parameters=common,
        ),
        Node(
            package='puppy_gait_cpp',
            executable='smart_home_manager',
            name='smart_home_manager',
            output='screen',
            parameters=common,
        ),
        Node(
            package='puppy_gait_cpp',
            executable='task_scheduler',
            name='task_scheduler',
            output='screen',
            parameters=common,
        ),
        Node(
            package='puppy_gait_cpp',
            executable='emergency_response',
            name='emergency_response',
            output='screen',
            parameters=common,
        ),
    ])

