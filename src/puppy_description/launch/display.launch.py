import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('puppy_description')
    default_rviz = os.path.join(pkg_share, 'rviz', 'display.rviz')
    default_urdf = os.path.join(pkg_share, 'urdf', 'puppy.urdf.xacro')

    rviz_arg = DeclareLaunchArgument('rvizconfig',
                                     default_value=default_rviz,
                                     description='RViz config file')

    urdf_arg = DeclareLaunchArgument('urdf',
                                     default_value=default_urdf,
                                     description='URDF xacro file')

    robot_description_content = Command(
        ['xacro ', LaunchConfiguration('urdf')])

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content,
                     'use_sim_time': True}],
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        rviz_arg,
        urdf_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz2,
    ])
