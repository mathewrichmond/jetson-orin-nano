#!/usr/bin/env python3
"""
Launch file for PHAT Motor Controller
"""

# Standard library
import os

# Third-party
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def find_config_file():
    """Find config file, checking centralized config first"""
    # Check centralized config
    centralized_config = os.path.join(
        os.getenv("ISAAC_ROOT", "/home/nano/src/jetson-orin-nano"),
        "config",
        "hardware",
        "phat_params.yaml",
    )
    if os.path.exists(centralized_config):
        return centralized_config

    # Fallback to package config
    package_config = PathJoinSubstitution(
        [FindPackageShare("phat_motor_controller"), "config", "phat_params.yaml"]
    )
    return package_config


def generate_launch_description():
    config_file = find_config_file()

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=config_file,
                description="Path to parameter config file",
            ),
            Node(
                package="phat_motor_controller",
                executable="phat_motor_controller_node",
                name="phat_motor_controller",
                namespace="/hardware",
                parameters=[LaunchConfiguration("config_file")],
                output="screen",
            ),
        ]
    )
