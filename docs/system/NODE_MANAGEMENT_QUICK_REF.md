# Node Management Quick Reference

Quick reference for managing ROS 2 nodes in the Isaac robot system.

## Entry Points

All nodes have consistent entry points:

```bash
# System monitor
ros2 run system_monitor system_monitor_node

# RealSense cameras
ros2 run realsense_camera realsense_camera_node

# Hello world (example)
ros2 run hello_world hello_world_node
```

## Graph-Based Management

### Start Nodes

```bash
# Start from graph config
ros2 launch isaac_robot graph.launch.py \
    graph_config:=robot_graph.yaml \
    group:=all

# Or use management script
./scripts/system/manage_nodes.sh start robot_graph.yaml all
```

### Available Graphs

- `robot_graph.yaml` - Default configuration
- `minimal_graph.yaml` - System monitor only
- `full_graph.yaml` - All components

### Available Groups

- `core` - Essential nodes (system_monitor)
- `hardware` - Hardware drivers (realsense_camera)
- `control` - Control nodes (vla_controller)
- `all` - All enabled nodes

## Management Script

```bash
# Start
./scripts/system/manage_nodes.sh start [graph] [group]

# Stop
./scripts/system/manage_nodes.sh stop

# Status
./scripts/system/manage_nodes.sh status

# List nodes in graph
./scripts/system/manage_nodes.sh list [graph]

# Restart
./scripts/system/manage_nodes.sh restart [graph] [group]

# Logs
./scripts/system/manage_nodes.sh logs [node_name]
```

## Systemd Service

```bash
# Enable (start on boot)
sudo systemctl enable isaac-robot.service

# Start
sudo systemctl start isaac-robot.service

# Stop
sudo systemctl stop isaac-robot.service

# Status
sudo systemctl status isaac-robot.service

# Logs
sudo journalctl -u isaac-robot.service -f
```

## Verification

```bash
# Check running nodes
ros2 node list

# Check topics
ros2 topic list

# Check node info
ros2 node info /namespace/node_name

# Check entry points
ros2 pkg executables
```

## Adding New Node

1. Add entry point to `setup.py`
2. Add entry point script to `CMakeLists.txt`
3. Add to graph config YAML
4. Rebuild: `colcon build --packages-select package_name`
5. Test: `ros2 run package_name node_name`

See [Node Management Guide](NODE_MANAGEMENT.md) for details.
