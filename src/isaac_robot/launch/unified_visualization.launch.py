#!/usr/bin/env python3
"""
Unified visualization launch file
Supports multiple visualization backends:
- Foxglove Studio (desktop app, cross-platform)
- RViz2 (Linux native)
- Web-based via rosbridge (Foxglove Web, custom UIs)

This is a generic visualization launcher that can be used with any ROS 2 system.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import EqualsSubstitution
import os


def generate_launch_description():
    # Get RViz config file path (if exists)
    rviz_config_file = os.path.join(
        os.path.expanduser('~'),
        'ros2_ws',
        'install',
        'isaac_robot',
        'share',
        'isaac_robot',
        'config',
        'visualization.rviz'
    )

    # If not found in install space, try source space
    if not os.path.exists(rviz_config_file):
        rviz_config_file = os.path.join(
            os.path.dirname(__file__),
            '..',
            'config',
            'visualization.rviz'
        )
        if not os.path.exists(rviz_config_file):
            rviz_config_file = None

    return LaunchDescription([
        DeclareLaunchArgument(
            'viz_backend',
            default_value='rosbridge',
            choices=['rosbridge', 'rviz2', 'none'],
            description='Visualization backend: rosbridge (web), rviz2 (native), or none'
        ),
        DeclareLaunchArgument(
            'rosbridge_port',
            default_value='9090',
            description='Port for rosbridge_server WebSocket'
        ),
        DeclareLaunchArgument(
            'rosbridge_address',
            default_value='0.0.0.0',
            description='Address to bind rosbridge_server'
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=rviz_config_file if rviz_config_file else '',
            description='Path to RViz2 configuration file (optional)'
        ),

        # Rosbridge Server (for web-based visualization)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('isaac_robot'),
                    'launch',
                    'rosbridge.launch.py'
                ])
            ]),
            launch_arguments={
                'rosbridge_port': LaunchConfiguration('rosbridge_port'),
                'rosbridge_address': LaunchConfiguration('rosbridge_address'),
            }.items(),
            condition=IfCondition(
                EqualsSubstitution(LaunchConfiguration('viz_backend'), 'rosbridge')
            ),
        ),

        # RViz2 (native Linux visualization)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config')] if rviz_config_file else None,
            condition=IfCondition(
                EqualsSubstitution(LaunchConfiguration('viz_backend'), 'rviz2')
            ),
            output='screen',
        ),
    ])
