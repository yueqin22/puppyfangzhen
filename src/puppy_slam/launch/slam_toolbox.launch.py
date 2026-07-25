import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('puppy_slam')
    params_file = os.path.join(pkg_share, 'config',
                               'mapper_params_online_async.yaml')

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[params_file, {'use_sim_time': True}],
        remappings=[('scan', '/scan')],
        output='screen',
    )

    return LaunchDescription([
        slam_node,
    ])
