import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('puppy_gait')
    params_file = os.path.join(pkg_share, 'config', 'gait_params.yaml')

    gait_controller = Node(
        package='puppy_gait',
        executable='gait_controller',
        name='gait_controller',
        parameters=[params_file, {'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        gait_controller,
    ])
