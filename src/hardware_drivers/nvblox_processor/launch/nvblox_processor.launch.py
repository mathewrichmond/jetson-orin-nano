#!/usr/bin/env python3
"""
nvblox Processor Launch File
Launches nvblox processor node for RealSense data preprocessing
"""

# Standard library
import os

# Third-party
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def find_config_file():
    """Find nvblox config file in centralized config directory"""
    # Check centralized config first
    centralized_config = os.path.join(
        os.path.expanduser("~"),
        "src",
        "jetson-orin-nano",
        "config",
        "hardware",
        "nvblox_params.yaml",
    )
    if os.path.exists(centralized_config):
        return centralized_config

    # Fallback to package config
    package_config = os.path.join(
        os.path.dirname(__file__),
        "..",
        "config",
        "nvblox_params.yaml",
    )
    if os.path.exists(package_config):
        return os.path.abspath(package_config)

    return "nvblox_params.yaml"


def generate_launch_description():
    # Get config file path from centralized config
    config_file = find_config_file()

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file", default_value=config_file, description="Path to configuration file"
            ),
            Node(
                package="nvblox_processor",
                executable="nvblox_processor_node",
                name="nvblox_processor",
                parameters=[LaunchConfiguration("config_file")],
                output="screen",
            ),
        ]
    )
