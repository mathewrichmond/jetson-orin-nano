#!/usr/bin/env python3
"""
Minimal Robot Launch
Launches only essential nodes for basic operation
"""

# Standard library
import os

# Third-party
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    # Run system_monitor_node with ROS 2 environment sourced
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
            system_monitor_node,
        ]
    )
