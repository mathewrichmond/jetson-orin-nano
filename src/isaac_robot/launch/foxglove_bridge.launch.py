#!/usr/bin/env python3
"""
Foxglove Bridge Launch File
Provides native Foxglove Studio connection (better than rosbridge)
"""

# Third-party
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "port", default_value="8765", description="Port for Foxglove Bridge WebSocket"
            ),
            DeclareLaunchArgument(
                "address",
                default_value="0.0.0.0",
                description="Address to bind Foxglove Bridge (0.0.0.0 for all interfaces)",
            ),
            DeclareLaunchArgument(
                "tls", default_value="false", description="Enable TLS encryption"
            ),
            DeclareLaunchArgument(
                "certfile",
                default_value="",
                description="Path to certificate file (if TLS enabled)",
            ),
            DeclareLaunchArgument(
                "keyfile", default_value="", description="Path to key file (if TLS enabled)"
            ),
            # Foxglove Bridge Node
            # Note: Port needs to be integer, use default value directly
            # Address can be string from LaunchConfiguration
            Node(
                package="foxglove_bridge",
                executable="foxglove_bridge",
                name="foxglove_bridge",
                parameters=[
                    {
                        # Default port (can be changed via command line args if needed)
                        "port": 8765,
                        "address": LaunchConfiguration("address"),
                    }
                ],
                output="screen",
            ),
        ]
    )
