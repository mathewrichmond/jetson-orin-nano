#!/usr/bin/env python3
"""
Graph Manager
Loads graph configuration from YAML and launches nodes accordingly
Provides consistent node management for the Isaac robot system
"""

# Standard library
from pathlib import Path
from typing import Any, Dict, List, Optional

# Third-party
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node as LaunchNode
import yaml


class GraphManager:
    """Manages robot graph configuration and node launching"""

    def __init__(self, config_file: Optional[str] = None):
        """
        Initialize graph manager

        Args:
            config_file: Path to graph configuration YAML file
        """
        self.config_file = config_file
        self.config: Dict[str, Any] = {}
        self.nodes: Dict[str, Dict] = {}

    def load_config(self, config_file: Optional[str] = None) -> Dict[str, Any]:
        """
        Load graph configuration from YAML file

        Args:
            config_file: Path to config file (uses self.config_file if None)

        Returns:
            Loaded configuration dictionary
        """
        if config_file is None:
            config_file = self.config_file

        if config_file is None:
            raise ValueError("No config file specified")

        config_path = Path(config_file)

        # Try to find config file in various locations
        if not config_path.exists():
            # Try package share
            try:
                # Third-party
                from ament_index_python.packages import get_package_share_directory

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
            source_config = (
                Path(__file__).parent.parent.parent / "config" / "robot" / config_path.name
            )
            if source_config.exists():
                config_path = source_config

        if not config_path.exists():
            raise FileNotFoundError(f"Graph config file not found: {config_file}")

        with open(config_path, "r") as f:
            self.config = yaml.safe_load(f) or {}
            self.nodes = self.config.get("robot", {})

        return self.config

    def get_node_config(self, node_name: str) -> Optional[Dict]:
        """Get configuration for a specific node"""
        return self.nodes.get(node_name)

    def get_enabled_nodes(self, group: Optional[str] = None) -> List[str]:
        """
        Get list of enabled node names

        Args:
            group: Node group name (e.g., 'core', 'hardware', 'all')

        Returns:
            List of enabled node names
        """
        if group:
            groups = self.config.get("groups", {})
            if group not in groups:
                return []
            node_names = groups[group]
        else:
            node_names = list(self.nodes.keys())

        # Filter to only enabled nodes
        enabled = []
        for name in node_names:
            node_config = self.nodes.get(name, {})
            if node_config.get("enabled", True):
                enabled.append(name)

        return enabled

    def create_node_action(self, node_name: str, node_config: Dict) -> LaunchNode:
        """
        Create a ROS 2 launch Node action from node configuration

        Args:
            node_name: Name of the node
            node_config: Node configuration dictionary

        Returns:
            LaunchNode action
        """
        package = node_config.get("package")
        executable = node_config.get("node")
        namespace = node_config.get("namespace", "")
        parameters = node_config.get("parameters", {})

        if not package or not executable:
            raise ValueError(f"Node {node_name} missing package or node executable")

        # Convert parameters dict to list format for launch
        param_list = []
        if parameters:
            param_list.append(parameters)

        # Create remappings from topics if specified
        remappings = []
        topics = node_config.get("topics", {})
        if "remap" in topics:
            for remap in topics["remap"]:
                remappings.append((remap["from"], remap["to"]))

        return LaunchNode(
            package=package,
            executable=executable,
            name=node_name,
            namespace=namespace if namespace else None,
            parameters=param_list,
            remappings=remappings if remappings else None,
            output="screen",
        )

    def generate_launch_description(self, group: Optional[str] = None) -> LaunchDescription:
        """
        Generate launch description from graph configuration

        Args:
            group: Node group to launch (None = all enabled nodes)

        Returns:
            LaunchDescription with all configured nodes
        """
        if not self.config:
            self.load_config()

        enabled_nodes = self.get_enabled_nodes(group)

        actions = []

        # Add launch arguments
        actions.append(
            DeclareLaunchArgument(
                "graph_config",
                default_value=self.config_file or "robot_graph.yaml",
                description="Graph configuration file",
            )
        )

        actions.append(
            DeclareLaunchArgument(
                "group", default_value=group or "all", description="Node group to launch"
            )
        )

        # Create node actions
        for node_name in enabled_nodes:
            node_config = self.nodes[node_name]
            try:
                node_action = self.create_node_action(node_name, node_config)
                actions.append(node_action)
            except Exception as e:
                print(f"Warning: Failed to create node {node_name}: {e}")
                continue

        return LaunchDescription(actions)


def generate_launch_description_from_config(
    config_file: str, group: Optional[str] = None
) -> LaunchDescription:
    """
    Convenience function to generate launch description from config file

    Args:
        config_file: Path to graph configuration YAML
        group: Node group to launch

    Returns:
        LaunchDescription
    """
    manager = GraphManager(config_file)
    return manager.generate_launch_description(group)
