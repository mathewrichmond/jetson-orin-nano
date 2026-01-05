#!/usr/bin/env python3
"""
Generic rosbridge launch file for web-based visualization
Provides rosbridge_server for connecting web clients (Foxglove Web, custom UIs, etc.)
Can be used with any ROS 2 system
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'rosbridge_port',
            default_value='9090',
            description='Port for rosbridge_server WebSocket'
        ),
        DeclareLaunchArgument(
            'rosbridge_address',
            default_value='0.0.0.0',
            description='Address to bind rosbridge_server (0.0.0.0 for all interfaces)'
        ),
        DeclareLaunchArgument(
            'fragment_timeout',
            default_value='600.0',
            description='Fragment timeout in seconds'
        ),
        DeclareLaunchArgument(
            'delay_between_messages',
            default_value='0.0',
            description='Delay between messages in seconds'
        ),
        DeclareLaunchArgument(
            'max_message_size',
            default_value='None',
            description='Maximum message size (None for unlimited)'
        ),
        DeclareLaunchArgument(
            'unregister_timeout',
            default_value='10.0',
            description='Unregister timeout in seconds'
        ),

        # Rosbridge Server for web-based visualization
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[{
                'port': LaunchConfiguration('rosbridge_port'),
                'address': LaunchConfiguration('rosbridge_address'),
                'fragment_timeout': LaunchConfiguration('fragment_timeout'),
                'delay_between_messages': LaunchConfiguration('delay_between_messages'),
                'max_message_size': LaunchConfiguration('max_message_size'),
                'unregister_timeout': LaunchConfiguration('unregister_timeout'),
            }],
            output='screen',
        ),
    ])
