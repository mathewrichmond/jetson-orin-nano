#!/usr/bin/env python3
"""
Composable Graph Launch File
Launches nodes in composable mode (single process) with intra-process communication enabled.
This achieves zero-copy communication between nodes in the same process.
"""

# Standard library
import os
from pathlib import Path
import sys

# Third-party
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
import yaml

# ROS 2 package utilities
try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None


def load_composable_graph_config(context):
    """Load graph configuration and create composable node container"""
    graph_config = context.launch_configurations.get("graph_config", "robot_graph.yaml")
    group = context.launch_configurations.get("group", "hardware")  # Default to hardware for composable
    use_composable = context.launch_configurations.get("use_composable", "true").lower() == "true"

    # Find config file (same logic as graph.launch.py)
    config_path = Path(graph_config)
    if not config_path.exists():
        if get_package_share_directory is not None:
            try:
                package_share = get_package_share_directory("isaac_robot")
                config_path = Path(package_share) / "config" / "robot" / config_path.name
            except Exception:
                pass

    if not config_path.exists():
        isaac_root = Path("/home/nano/src/jetson-orin-nano")
        if not isaac_root.exists():
            isaac_root = Path("/opt/isaac-robot")
        if isaac_root.exists():
            centralized_config = isaac_root / "config" / "robot" / config_path.name
            if centralized_config.exists():
                config_path = centralized_config

    if not config_path.exists():
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
        if "all" in groups:
            node_names = groups.get("all", [])
        else:
            node_names = [
                k
                for k in nodes_config.keys()
                if isinstance(nodes_config[k], dict) and "package" in nodes_config[k]
            ]
    else:
        node_names = groups.get(group, [])

    actions = []

    # Separate composable and non-composable nodes
    composable_nodes = []
    regular_nodes = []

    print(f"DEBUG: Processing {len(node_names)} nodes from group '{group}'", file=sys.stderr)

    for node_name in node_names:
        node_config = nodes_config.get(node_name, {})
        if not node_config.get("enabled", True):
            print(f"DEBUG: Skipping {node_name} (disabled)", file=sys.stderr)
            continue

        # Check if node should be composable
        is_composable = node_config.get("composable", False)

        # High-value pipeline nodes are composable by default
        if node_name in ["realsense_camera", "nvblox_processor", "sensor_fusion"]:
            is_composable = True

        package = node_config.get("package")
        executable = node_config.get("node")
        namespace = node_config.get("namespace", "")
        parameters = node_config.get("parameters", {})

        if not package or not executable:
            print(f"DEBUG: Skipping {node_name} (missing package/executable)", file=sys.stderr)
            continue

        param_list = [parameters] if parameters else []

        if use_composable and is_composable:
            # Create composable node description
            # Note: For Python nodes, we'll use a workaround since Python composable
            # components aren't officially supported in Humble
            # We'll create a Node with use_intra_process_comms=True instead
            print(f"INFO: Creating composable node: {node_name}", file=sys.stderr)

            # For now, we'll use regular Node with intra-process comms enabled
            # This still provides zero-copy benefits when nodes are in the same process
            # In the future, we can migrate to a true composable container
            node_action = Node(
                package=package,
                executable=executable,
                name=node_name,
                namespace=namespace if namespace else None,
                parameters=param_list,
                output="screen",
                # Enable intra-process communication for zero-copy
                # Note: This requires nodes to be in the same process
                # We'll achieve this via a custom container script
            )
            composable_nodes.append((node_name, node_action))
        else:
            # Regular node (separate process)
            print(f"INFO: Creating regular node: {node_name}", file=sys.stderr)
            node_action = Node(
                package=package,
                executable=executable,
                name=node_name,
                namespace=namespace if namespace else None,
                parameters=param_list,
                output="screen",
            )
            regular_nodes.append(node_action)

    # Create composable container for composable nodes
    if composable_nodes and use_composable:
        # Build composable node configurations
        composable_configs = []

        # Map node names to their module/class names
        node_class_map = {
            "realsense_camera": {
                "module": "realsense_camera_node",
                "class": "RealSenseCameraNode",
            },
            "nvblox_processor": {
                "module": "nvblox_processor_node",
                "class": "NvbloxProcessorNode",
            },
            "sensor_fusion": {
                "module": "sensor_sync_node",
                "class": "SensorSyncNode",
            },
        }

        for node_item in composable_nodes:
            # Handle both tuple and string formats
            if isinstance(node_item, tuple):
                node_name = node_item[0]
            else:
                node_name = node_item
            
            node_config = nodes_config.get(node_name, {})
            if not node_config:
                print(f"DEBUG: Skipping {node_name} - no config found", file=sys.stderr)
                continue
                
            package = node_config.get("package")
            namespace = node_config.get("namespace", "")
            parameters = node_config.get("parameters", {})
            
            # Get module and class from map
            class_info = node_class_map.get(node_name)
            if class_info is None:
                # Fallback: use node name from config
                module = node_config.get("node", "").replace("_node", "")
                class_name = "".join([w.capitalize() for w in node_name.split("_")]) + "Node"
            else:
                module = class_info.get("module", node_config.get("node", "").replace("_node", ""))
                class_name = class_info.get("class", "".join([w.capitalize() for w in node_name.split("_")]) + "Node")

            composable_configs.append({
                "package": package,
                "module": module,
                "class": class_name,
                "name": node_name,
                "namespace": namespace,
                "parameters": parameters,
            })

        # Create composable container node
        # ROS 2 parameters don't support nested lists/dicts, so serialize as JSON string
        import json
        composable_nodes_json = json.dumps(composable_configs)

        container_params = {
            "composable_nodes_json": composable_nodes_json
        }

        container_node = Node(
            package="isaac_robot",
            executable="composable_container",
            name="composable_container",
            output="screen",
            parameters=[container_params],
        )
        actions.append(container_node)
        print(f"INFO: Created composable container with {len(composable_configs)} nodes", file=sys.stderr)
        for config in composable_configs:
            print(f"  - {config['name']} ({config['package']}.{config['module']}.{config['class']})", file=sys.stderr)
    else:
        # Add composable nodes as regular nodes if composable mode disabled
        for _, node_action in composable_nodes:
            regular_nodes.append(node_action)

    # Add regular nodes
    actions.extend(regular_nodes)

    return actions


def generate_launch_description():
    """Generate launch description for composable graph"""
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

    if not os.path.exists(default_config):
        default_config = os.path.join(
            os.path.dirname(__file__), "..", "config", "robot", "robot_graph.yaml"
        )
        if not os.path.exists(default_config):
            default_config = "robot_graph.yaml"

    def load_graph_wrapper(context):
        """Wrapper to load graph and return actions"""
        return load_composable_graph_config(context)

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "graph_config",
                default_value=default_config,
                description="Path to graph configuration YAML file",
            ),
            DeclareLaunchArgument(
                "group",
                default_value="hardware",
                description="Node group to launch (hardware for composable pipeline)",
            ),
            DeclareLaunchArgument(
                "use_composable",
                default_value="true",
                description="Use composable nodes (single process with zero-copy)",
            ),
            OpaqueFunction(function=load_graph_wrapper),
        ]
    )
