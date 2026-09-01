"""Launch file for MiniCPM-RobotTrack Go2 real robot deployment with safety guards."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory("puppy_minicpm_robot") if "puppy_minicpm_robot" in os.environ.get("AMENT_PREFIX_PATH", "") else os.path.dirname(os.path.dirname(__file__))
    default_config_path = os.path.join(pkg_share, "config", "minicpm_robot.yaml")

    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="dry-run",
        description="Execution mode: dry-run (safe default), sim, or live-arm (requires explicit confirmation)"
    )
    backend_arg = DeclareLaunchArgument(
        "backend",
        default_value="http",
        description="Inference backend: http (connecting to local Orin server) or mock"
    )
    camera_source_arg = DeclareLaunchArgument(
        "camera_source",
        default_value="go2",
        description="Camera source: go2 or realsense"
    )
    server_url_arg = DeclareLaunchArgument(
        "server_url",
        default_value="http://127.0.0.1:5801",
        description="MiniCPM-RobotTrack inference server URL"
    )
    instruction_arg = DeclareLaunchArgument(
        "instruction",
        default_value="Follow the person ahead",
        description="Initial natural language tracking instruction"
    )

    mode = LaunchConfiguration("mode")
    backend = LaunchConfiguration("backend")
    camera_source = LaunchConfiguration("camera_source")
    server_url = LaunchConfiguration("server_url")
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
            {"mode": mode, "backend": backend, "server_url": server_url, "instruction": instruction}
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
        LogInfo(msg="[WARNING] Go2 deployment initialized. Ensure emergency stop is reachable!"),
        mode_arg,
        backend_arg,
        camera_source_arg,
        server_url_arg,
        instruction_arg,
        vision_bridge_node,
        minicpm_track_node,
        track_cmd_adapter_node,
        mission_grounder_node
    ])
