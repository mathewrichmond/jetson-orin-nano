#!/usr/bin/env python3
"""
Graph-based Launch File
Launches nodes from graph configuration YAML file
Uses GraphManager to dynamically load and launch nodes from YAML config
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import os
import yaml
from pathlib import Path
from launch_ros.actions import Node


def load_graph_config(context):
    """Load graph configuration and return node actions"""
    graph_config = context.launch_configurations.get('graph_config', 'robot_graph.yaml')
    group = context.launch_configurations.get('group', 'all')
    
    # Find config file
    config_path = Path(graph_config)
    if not config_path.exists():
        # Try package share
        try:
            from ament_index_python.packages import get_package_share_directory
            package_share = get_package_share_directory('isaac_robot')
            config_path = Path(package_share) / 'config' / 'robot' / config_path.name
        except:
            pass
    
    if not config_path.exists():
        # Try source space
        source_config = Path(__file__).parent.parent / 'config' / 'robot' / config_path.name
        if source_config.exists():
            config_path = source_config
    
    if not config_path.exists():
        raise FileNotFoundError(f"Graph config file not found: {graph_config}")
    
    # Load config
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f) or {}
    
    nodes_config = config.get('robot', {})
    groups = config.get('groups', {})
    
    # Get nodes to launch
    if group == 'all':
        node_names = list(nodes_config.keys())
    else:
        node_names = groups.get(group, [])
    
    # Create node actions
    actions = []
    for node_name in node_names:
        node_config = nodes_config.get(node_name, {})
        if not node_config.get('enabled', True):
            continue
        
        package = node_config.get('package')
        executable = node_config.get('node')
        namespace = node_config.get('namespace', '')
        parameters = node_config.get('parameters', {})
        
        if not package or not executable:
            continue
        
        param_list = [parameters] if parameters else []
        
        actions.append(Node(
            package=package,
            executable=executable,
            name=node_name,
            namespace=namespace if namespace else None,
            parameters=param_list,
            output='screen',
        ))
    
    return actions


def generate_launch_description():
    # Get default config file path
    default_config = os.path.join(
        os.path.expanduser('~'),
        'ros2_ws',
        'install',
        'isaac_robot',
        'share',
        'isaac_robot',
        'config',
        'robot',
        'robot_graph.yaml'
    )
    
    # Try source space if install space doesn't exist
    if not os.path.exists(default_config):
        default_config = os.path.join(
            os.path.dirname(__file__),
            '..',
            'config',
            'robot',
            'robot_graph.yaml'
        )
        if not os.path.exists(default_config):
            default_config = 'robot_graph.yaml'
    
    def load_graph_wrapper(context):
        """Wrapper to load graph and return actions"""
        return load_graph_config(context)
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'graph_config',
            default_value=default_config,
            description='Path to graph configuration YAML file'
        ),
        DeclareLaunchArgument(
            'group',
            default_value='all',
            description='Node group to launch (core, hardware, control, all)'
        ),
        OpaqueFunction(function=load_graph_wrapper),
    ])

