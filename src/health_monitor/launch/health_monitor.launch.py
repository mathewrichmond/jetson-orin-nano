#!/usr/bin/env python3
"""
Health Monitor Launch File
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
                "graph_config", default_value="robot_graph.yaml", description="Graph config file"
            ),
            DeclareLaunchArgument("group", default_value="all", description="Graph group"),
            DeclareLaunchArgument(
                "health_publish_rate", default_value="1.0", description="Health publish rate (Hz)"
            ),
            Node(
                package="health_monitor",
                executable="health_monitor_node",
                name="health_monitor",
                output="screen",
                parameters=[
                    {"graph_config": LaunchConfiguration("graph_config")},
                    {"group": LaunchConfiguration("group")},
                    {"health_publish_rate": LaunchConfiguration("health_publish_rate")},
                ],
            ),
        ]
    )
