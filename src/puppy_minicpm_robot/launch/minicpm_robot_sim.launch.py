"""Launch file for MiniCPM-RobotTrack simulation environment."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("puppy_minicpm_robot") if "puppy_minicpm_robot" in os.environ.get("AMENT_PREFIX_PATH", "") else os.path.dirname(os.path.dirname(__file__))
    default_config_path = os.path.join(pkg_share, "config", "minicpm_robot.yaml")

    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="dry-run",
        description="Execution mode: dry-run, sim, or live-arm"
    )
    backend_arg = DeclareLaunchArgument(
        "backend",
        default_value="mock",
        description="Inference backend: mock or http"
    )
    camera_source_arg = DeclareLaunchArgument(
        "camera_source",
        default_value="synthetic",
        description="Camera source: sim, realsense, go2, or synthetic"
    )
    instruction_arg = DeclareLaunchArgument(
        "instruction",
        default_value="Follow the person ahead",
        description="Default natural language tracking instruction"
    )

    mode = LaunchConfiguration("mode")
    backend = LaunchConfiguration("backend")
    camera_source = LaunchConfiguration("camera_source")
    instruction = LaunchConfiguration("instruction")

    vision_bridge_node = Node(
        package="puppy_minicpm_robot",
        executable="vision_bridge_node",
        name="vision_bridge_node",
        output="screen",
        parameters=[
            default_config_path,
            {"camera_source": camera_source}
        ]
    )

    minicpm_track_node = Node(
        package="puppy_minicpm_robot",
        executable="minicpm_track_node",
        name="minicpm_track_node",
        output="screen",
        parameters=[
            default_config_path,
            {"mode": mode, "backend": backend, "instruction": instruction}
        ]
    )

    track_cmd_adapter_node = Node(
        package="puppy_minicpm_robot",
        executable="track_cmd_adapter_node",
        name="track_cmd_adapter_node",
        output="screen",
        parameters=[
            default_config_path,
            {"mode": mode}
        ]
    )

    mission_grounder_node = Node(
        package="puppy_minicpm_robot",
        executable="mission_grounder_node",
        name="mission_grounder_node",
        output="screen",
        parameters=[default_config_path]
    )

    return LaunchDescription([
        mode_arg,
        backend_arg,
        camera_source_arg,
        instruction_arg,
        vision_bridge_node,
        minicpm_track_node,
        track_cmd_adapter_node,
        mission_grounder_node
    ])
