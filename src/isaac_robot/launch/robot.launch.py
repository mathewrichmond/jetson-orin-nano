#!/usr/bin/env python3
"""
Main Robot Launch File
Launches the complete robot system based on graph configuration
"""

# Standard library
import os
from pathlib import Path

# Third-party
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.substitutions import FindPackageShare
import yaml


def load_graph_config(config_file: str) -> dict:
    """Load robot graph configuration from YAML file"""
    config_path = Path(config_file)
    if not config_path.exists():
        # Try to find in package share
        package_share = FindPackageShare("isaac_robot").find("isaac_robot")
        config_path = Path(package_share) / "config" / "robot" / config_file

    if not config_path.exists():
        raise FileNotFoundError(f"Graph config file not found: {config_file}")

    with open(config_path, "r") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    # Launch arguments
    config_arg = DeclareLaunchArgument(
        "graph_config",
        default_value="robot_graph.yaml",
        description="Robot graph configuration file",
    )

    group_arg = DeclareLaunchArgument(
        "group",
        default_value="core",
        description="Node group to launch (core, control, hardware, all)",
    )

    # Load graph configuration
    # Note: In ROS 2 launch, we need to use a Python function to load config
    # For now, we'll use a simpler approach with launch arguments

    # System Monitor (always enabled in core group)
    script_path = os.path.expanduser(
        "~/ros2_ws/install/system_monitor/local/lib/python3.10/"
        "dist-packages/system_monitor/system_monitor_node.py"
    )
    system_monitor_node = ExecuteProcess(
        cmd=[
            "/bin/bash",
            "-c",
            (
                f"source /opt/ros/humble/setup.bash && "
                f"source ~/ros2_ws/install/setup.bash && "
                f"python3 {script_path}"
            ),
        ],
        name="system_monitor",
        output="screen",
        env={
            "HOME": os.path.expanduser("~"),
            "ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID", "0"),
            **os.environ,
        },
    )

    return LaunchDescription(
        [
            config_arg,
            group_arg,
            system_monitor_node,
        ]
    )


if __name__ == "__main__":
    generate_launch_description()
