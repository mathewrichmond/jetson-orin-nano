#!/usr/bin/env python3
"""
Graph-based Launch File
Launches nodes from graph configuration YAML file
Uses GraphManager to dynamically load and launch nodes from YAML config
"""

# Standard library
import os
from pathlib import Path
import sys

# Third-party
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
import yaml

# ROS 2 package utilities (may not be available in all environments)
try:
    # Third-party
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None


def load_graph_config(context):
    """Load graph configuration and return node actions"""
    graph_config = context.launch_configurations.get("graph_config", "robot_graph.yaml")
    group = context.launch_configurations.get("group", "all")

    # Find config file
    config_path = Path(graph_config)
    if not config_path.exists():
        # Try package share
        if get_package_share_directory is not None:
            try:
                package_share = get_package_share_directory("isaac_robot")
                config_path = Path(package_share) / "config" / "robot" / config_path.name
            except Exception:
                pass

    if not config_path.exists():
        # Try centralized config directory
        isaac_root = Path("/home/nano/src/jetson-orin-nano")
        if not isaac_root.exists():
            isaac_root = Path("/opt/isaac-robot")

        if isaac_root.exists():
            centralized_config = isaac_root / "config" / "robot" / config_path.name
            if centralized_config.exists():
                config_path = centralized_config

    if not config_path.exists():
        # Try source space (legacy)
        source_config = Path(__file__).parent.parent / "config" / "robot" / config_path.name
        if source_config.exists():
            config_path = source_config

    if not config_path.exists():
        raise FileNotFoundError(f"Graph config file not found: {graph_config}")

    # Load config
    with open(config_path, "r") as f:
        config = yaml.safe_load(f) or {}

    nodes_config = config.get("robot", {})
    groups = config.get("groups", {})

    # Get nodes to launch
    if group == "all":
        # Use the 'all' group list if available
        if "all" in groups:
            node_names = groups.get("all", [])
        else:
            # Fallback: get all node names, filtering out non-node entries
            node_names = [
                k
                for k in nodes_config.keys()
                if isinstance(nodes_config[k], dict) and "package" in nodes_config[k]
            ]
    else:
        node_names = groups.get(group, [])

    # Create node actions
    actions = []
    print(f"DEBUG: Processing {len(node_names)} nodes from group '{group}'", file=sys.stderr)
    for node_name in node_names:
        node_config = nodes_config.get(node_name, {})
        if not node_config.get("enabled", True):
            print(f"DEBUG: Skipping {node_name} (disabled)", file=sys.stderr)
            continue

        package = node_config.get("package")
        executable = node_config.get("node")
        namespace = node_config.get("namespace", "")
        parameters = node_config.get("parameters", {})

        if not package or not executable:
            print(f"DEBUG: Skipping {node_name} (missing package/executable)", file=sys.stderr)
            continue

        param_list = [parameters] if parameters else []

        # Debug: Print parameters for microphone node
        if node_name == "usb_microphone":
            print(f"DEBUG: USB Microphone parameters: {parameters}", file=sys.stderr)
            print(
                f"DEBUG: Channels value: {parameters.get('channels', 'NOT FOUND')}", file=sys.stderr
            )

        # Try to find executable - ROS 2 will handle this automatically
        # but we can add fallback for Python modules if needed
        try:
            node_action = Node(
                package=package,
                executable=executable,
                name=node_name,
                namespace=namespace if namespace else None,
                parameters=param_list,
                output="screen",
            )
            actions.append(node_action)
            print(
                f"INFO: Created node action for {node_name} ({package}/{executable})",
                file=sys.stderr,
            )
        except Exception as e:
            # If Node creation fails, log warning but continue
            # This allows the launch file to work even if some executables aren't found
            print(
                f"Warning: Could not create node {node_name} ({package}/{executable}): {e}",
                file=sys.stderr,
            )
            continue

    return actions


def generate_launch_description():
    # Get default config file path
    default_config = os.path.join(
        os.path.expanduser("~"),
        "ros2_ws",
        "install",
        "isaac_robot",
        "share",
        "isaac_robot",
        "config",
        "robot",
        "robot_graph.yaml",
    )

    # Try source space if install space doesn't exist
    if not os.path.exists(default_config):
        default_config = os.path.join(
            os.path.dirname(__file__), "..", "config", "robot", "robot_graph.yaml"
        )
        if not os.path.exists(default_config):
            default_config = "robot_graph.yaml"

    def load_graph_wrapper(context):
        """Wrapper to load graph and return actions"""
        return load_graph_config(context)

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "graph_config",
                default_value=default_config,
                description="Path to graph configuration YAML file",
            ),
            DeclareLaunchArgument(
                "group",
                default_value="all",
                description="Node group to launch (core, hardware, control, all)",
            ),
            OpaqueFunction(function=load_graph_wrapper),
        ]
    )
