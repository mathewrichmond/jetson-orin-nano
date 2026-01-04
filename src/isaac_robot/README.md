# Isaac Robot Package

Main robot launch and graph configuration package for the Isaac robot system.

## Overview

This package provides:
- Main robot launch files
- Graph configuration files (YAML)
- Robot system topology definitions

## Launch Files

### Minimal Launch
Launches only essential nodes:
```bash
ros2 launch isaac_robot minimal.launch.py
```

### Full Launch
Launches complete robot system:
```bash
ros2 launch isaac_robot full.launch.py
```

### Robot Launch (with config)
Launches with custom graph configuration:
```bash
ros2 launch isaac_robot robot.launch.py graph_config:=robot_graph.yaml group:=core
```

## Graph Configurations

Graph configurations define the robot system topology:

- `robot_graph.yaml` - Default robot configuration
- `minimal_graph.yaml` - Minimal system (system monitor only)
- `full_graph.yaml` - Full system with all components

### Configuration Structure

```yaml
robot:
  system_monitor:
    enabled: true
    package: system_monitor
    node: system_monitor_node
    namespace: /system
    parameters:
      update_rate: 1.0
    topics:
      publish:
        - /system/status
      subscribe: []
```

## Node Groups

- `core` - Essential nodes (system_monitor)
- `control` - Control nodes (vla_controller)
- `hardware` - Hardware drivers
- `all` - All nodes

## Usage

```bash
# Launch minimal system
ros2 launch isaac_robot minimal.launch.py

# Launch full system
ros2 launch isaac_robot full.launch.py

# Check running nodes
ros2 node list

# Check topics
ros2 topic list

# Monitor system status
ros2 topic echo /system/status
```

## Adding New Nodes

1. Add node definition to graph config YAML
2. Add node to appropriate launch file
3. Update node groups as needed
4. Rebuild package: `colcon build --packages-select isaac_robot`
