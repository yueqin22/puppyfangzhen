import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('puppy_gait')
    params_file = os.path.join(pkg_share, 'config', 'gait_params.yaml')
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')

    gait_controller = Node(
        package='puppy_gait',
        executable='gait_controller',
        name='gait_controller',
        parameters=[params_file, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
    )

    return LaunchDescription([
        use_sim_time, gait_controller,
    ])
