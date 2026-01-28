#!/usr/bin/env python3
"""
Distributed Deployment Launch File

Launches nodes for distributed deployment across multiple hosts.
Automatically selects the correct graph based on hostname.
"""

import os
import socket
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for distributed deployment"""
    
    # Declare launch arguments
    deployment_arg = DeclareLaunchArgument(
        'deployment',
        default_value='dual_compute',
        description='Deployment configuration (dual_compute, cloud_dev)'
    )
    
    host_arg = DeclareLaunchArgument(
        'host',
        default_value='auto',
        description='Host role (auto, pi, jetson, dev_workstation, robot)'
    )
    
    # Get hostname to auto-detect role
    hostname = socket.gethostname()
    
    # Determine host role
    if 'pi' in hostname.lower():
        default_graph = 'pi_graph.yaml'
        default_host = 'pi'
    elif 'jetson' in hostname.lower() or hostname == 'isaac':
        default_graph = 'jetson_graph.yaml'
        default_host = 'jetson'
    else:
        default_graph = 'modular_graph.yaml'
        default_host = 'unknown'
    
    graph_arg = DeclareLaunchArgument(
        'graph',
        default_value=default_graph,
        description='Graph configuration file'
    )
    
    group_arg = DeclareLaunchArgument(
        'group',
        default_value='all',
        description='Node group to launch'
    )
    
    # Get launch configurations
    deployment = LaunchConfiguration('deployment')
    host = LaunchConfiguration('host')
    graph = LaunchConfiguration('graph')
    group = LaunchConfiguration('group')
    
    # Include composable graph launch
    composable_graph_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                os.path.dirname(__file__),
                'composable_graph.launch.py'
            )
        ]),
        launch_arguments={
            'graph_config': graph,
            'group': group,
            'namespace': '',
        }.items()
    )
    
    return LaunchDescription([
        deployment_arg,
        host_arg,
        graph_arg,
        group_arg,
        composable_graph_launch,
    ])
