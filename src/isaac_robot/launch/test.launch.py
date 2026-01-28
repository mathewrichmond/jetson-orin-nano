#!/usr/bin/env python3
"""
Test Environment Launch File

Launches nodes in test mode with mock hardware.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for test environment"""
    
    # Launch arguments
    mock_hardware_arg = DeclareLaunchArgument(
        'mock_hardware',
        default_value='true',
        description='Enable mock hardware mode'
    )
    
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='true',
        description='Run in headless mode (no GUI)'
    )
    
    ros_domain_id_arg = DeclareLaunchArgument(
        'ros_domain_id',
        default_value='42',
        description='ROS domain ID for test environment'
    )
    
    # Get configurations
    mock_hardware = LaunchConfiguration('mock_hardware')
    headless = LaunchConfiguration('headless')
    ros_domain_id = LaunchConfiguration('ros_domain_id')
    
    # Set ROS domain ID for test isolation
    os.environ['ROS_DOMAIN_ID'] = '42'
    
    # Mock hardware nodes (placeholder - would launch mock drivers)
    mock_camera = Node(
        package='isaac_utils',
        executable='mock_camera_node',
        name='mock_camera',
        parameters=[{
            'mock_mode': mock_hardware,
            'image_topic': '/hardware/camera_front/color/image_raw',
            'fps': 30,
        }],
        condition=lambda context: context.launch_configurations['mock_hardware'] == 'true'
    )
    
    # Load main graph with test parameters
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    
    main_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                os.path.dirname(__file__),
                'graph.launch.py'
            )
        ]),
        launch_arguments={
            'graph': 'modular_graph.yaml',
            'group': 'all',
            'deployment': 'test',
        }.items()
    )
    
    return LaunchDescription([
        mock_hardware_arg,
        headless_arg,
        ros_domain_id_arg,
        main_launch,
    ])
