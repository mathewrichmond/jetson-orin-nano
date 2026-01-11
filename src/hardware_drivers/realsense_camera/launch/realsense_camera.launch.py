#!/usr/bin/env python3
"""
Launch file for RealSense camera nodes
"""

# Standard library
from pathlib import Path

# Third-party
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def find_config_file():
    """Find config file in centralized config/ directory"""
    # Try centralized config first
    isaac_root = Path("/home/nano/src/jetson-orin-nano")
    if not isaac_root.exists():
        isaac_root = Path("/opt/isaac-robot")

    if isaac_root.exists():
        config_file = isaac_root / "config" / "hardware" / "realsense_params.yaml"
        if config_file.exists():
            return str(config_file)

    # Fallback to package config
    try:
        # Third-party
        from ament_index_python.packages import get_package_share_directory

        pkg_share = get_package_share_directory("realsense_camera")
        config_file = Path(pkg_share) / "config" / "realsense_params.yaml"
        if config_file.exists():
            return str(config_file)
    except Exception:
        pass

    # Last resort: source space
    config_file = Path(__file__).parent.parent / "config" / "realsense_params.yaml"
    if config_file.exists():
        return str(config_file.absolute())

    return "realsense_params.yaml"


def generate_launch_description():
    # Get config file path from centralized config
    config_file = find_config_file()

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file", default_value=config_file, description="Path to configuration file"
            ),
            Node(
                package="realsense_camera",
                executable="realsense_camera_node",
                name="realsense_camera",
                parameters=[LaunchConfiguration("config_file")],
                output="screen",
                # Workaround for symlink installs - use Python module directly
                # if executable not found. This will be handled automatically
                # by ROS 2, but explicit for clarity
            ),
        ]
    )
