import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('puppy_localization')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')

    leg_odometry = Node(
        package='puppy_localization',
        executable='leg_odometry',
        name='leg_odometry',
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_config, {'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        leg_odometry,
        ekf,
    ])
