"""Mock bringup: fake motion + status + sensors for hardware-free development.

Use this launch file to test puppy_core, mission system, and safety
logic without PuppyPi hardware (gaijin2.md section 5.3.1).
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='puppypi_mock',
            executable='fake_motion',
            name='fake_motion_server',
            output='screen',
        ),
        Node(
            package='puppypi_mock',
            executable='fake_status',
            name='fake_status_server',
            output='screen',
        ),
        Node(
            package='puppypi_mock',
            executable='fake_sensors',
            name='fake_sensors',
            output='screen',
        ),
    ])
