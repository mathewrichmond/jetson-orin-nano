#!/usr/bin/env python3
"""
Full Robot Launch
Launches complete robot system with all components
"""

# Standard library
import os

# Third-party
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    # System Monitor
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

    # VLA Controller (placeholder - will be enabled when package exists)
    # vla_controller_node = Node(
    #     package='vla_controller',
    #     executable='vla_controller_node',
    #     name='vla_controller',
    #     namespace='control',
    #     output='screen',
    # )

    # RealSense Cameras
    realsense_node = ExecuteProcess(
        cmd=[
            "/bin/bash",
            "-c",
            (
                "source /opt/ros/humble/setup.bash && "
                "source ~/ros2_ws/install/setup.bash && "
                "ros2 run realsense_camera realsense_camera_node"
            ),
        ],
        name="realsense_camera",
        output="screen",
        env={
            "HOME": os.path.expanduser("~"),
            "ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID", "0"),
            **os.environ,
        },
    )

    # VLA Controller (placeholder - will be enabled when package exists)
    # vla_controller_node = Node(
    #     package='vla_controller',
    #     executable='vla_controller_node',
    #     name='vla_controller',
    #     namespace='control',
    #     output='screen',
    # )

    return LaunchDescription(
        [
            system_monitor_node,
            realsense_node,
            # vla_controller_node,
        ]
    )
