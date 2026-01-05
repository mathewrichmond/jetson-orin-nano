#!/usr/bin/env python3
"""
Launch file for RealSense camera nodes
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    # Get config file path
    config_file = os.path.join(
        os.path.expanduser('~'),
        'ros2_ws',
        'install',
        'realsense_camera',
        'share',
        'realsense_camera',
        'config',
        'realsense_params.yaml'
    )

    # If not found in install space, try source space
    if not os.path.exists(config_file):
        config_file = os.path.join(
            os.path.dirname(__file__),
            '..',
            'config',
            'realsense_params.yaml'
        )
        config_file = os.path.abspath(config_file)

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=config_file,
            description='Path to configuration file'
        ),

        Node(
            package='realsense_camera',
            executable='realsense_camera_node',
            name='realsense_camera',
            parameters=[LaunchConfiguration('config_file')],
            output='screen',
            # Workaround for symlink installs - use Python module directly if executable not found
            # This will be handled automatically by ROS 2, but explicit for clarity
        ),
    ])
