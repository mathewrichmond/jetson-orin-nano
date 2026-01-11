#!/usr/bin/env python3
"""
Generic rosbridge launch file for web-based visualization
Provides rosbridge_server for connecting web clients (Foxglove Web, custom UIs, etc.)
Can be used with any ROS 2 system
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
                "rosbridge_port",
                default_value="9090",
                description="Port for rosbridge_server WebSocket",
            ),
            DeclareLaunchArgument(
                "rosbridge_address",
                default_value="0.0.0.0",
                description="Address to bind rosbridge_server (0.0.0.0 for all interfaces)",
            ),
            DeclareLaunchArgument(
                "fragment_timeout",
                default_value="600",
                description="Fragment timeout in seconds (integer)",
            ),
            DeclareLaunchArgument(
                "delay_between_messages",
                default_value="0.0",
                description="Delay between messages in seconds",
            ),
            DeclareLaunchArgument(
                "max_message_size",
                default_value="-1",
                description="Maximum message size (-1 for unlimited)",
            ),
            DeclareLaunchArgument(
                "unregister_timeout",
                default_value="10.0",
                description="Unregister timeout in seconds",
            ),
            # Rosbridge Server for web-based visualization
            # Note: Parameters are passed as strings and rosbridge converts them
            # For integer parameters, we use integer defaults
            Node(
                package="rosbridge_server",
                executable="rosbridge_websocket",
                name="rosbridge_websocket",
                parameters=[
                    {
                        "port": 9090,  # Use integer directly
                        "address": LaunchConfiguration("rosbridge_address"),
                        "fragment_timeout": 600,  # Use integer directly
                        "delay_between_messages": 0.0,
                        "max_message_size": -1,  # -1 means unlimited
                        "unregister_timeout": 10.0,
                    }
                ],
                output="screen",
            ),
        ]
    )
