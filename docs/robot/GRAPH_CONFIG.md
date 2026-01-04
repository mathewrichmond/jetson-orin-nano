# Robot Graph Configuration

## Overview

The robot graph configuration system defines the complete topology of the Isaac robot system, including all nodes, their connections, parameters, and topic mappings.

## Configuration Files

Graph configurations are stored in `config/robot/`:

- **`robot_graph.yaml`** - Default robot configuration
- **`minimal_graph.yaml`** - Minimal system (system monitor only)
- **`full_graph.yaml`** - Full system with all components

## Configuration Structure

```yaml
robot:
  system_monitor:
    enabled: true
    package: system_monitor
    node: system_monitor_node
    namespace: /system
    parameters:
      update_rate: 1.0
      temp_warning_threshold: 70.0
    topics:
      publish:
        - /system/status
        - /system/temperature/cpu
      subscribe: []
```

### Node Definition

Each node in the graph has:

- **`enabled`** - Whether the node should be launched
- **`package`** - ROS 2 package name
- **`node`** - Node executable name
- **`namespace`** - ROS 2 namespace
- **`parameters`** - Node parameters (key-value pairs)
- **`topics`** - Topic mappings
  - **`publish`** - List of topics published by this node
  - **`subscribe`** - List of topics subscribed by this node

### Node Groups

Groups allow launching subsets of nodes:

```yaml
groups:
  core:
    - system_monitor
  control:
    - vla_controller
  all:
    - system_monitor
    - vla_controller
    - hardware_drivers
```

## Launch Files

The `isaac_robot` package provides launch files that use these configurations:

### Minimal Launch
```bash
ros2 launch isaac_robot minimal.launch.py
```
Launches only essential nodes (system monitor).

### Full Launch
```bash
ros2 launch isaac_robot full.launch.py
```
Launches complete robot system.

### Robot Launch (with config)
```bash
ros2 launch isaac_robot robot.launch.py \
    graph_config:=robot_graph.yaml \
    group:=core
```

## Current Graph

### Minimal Graph (Current)

```
┌─────────────────┐
│ system_monitor  │
│                 │
│ Publishes:      │
│ - /system/status│
│ - /system/temp  │
│ - /system/cpu   │
│ - /system/mem   │
└─────────────────┘
```

### Full Graph (Planned)

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│ hardware_drivers│────>│ vla_controller  │────>│  motor_control  │
│                 │     │                 │     │                 │
│ - Camera        │     │ - VLA Model     │     │ - Motor Cmds    │
│ - Sensors       │     │ - Planning      │     │ - Status        │
└─────────────────┘     └─────────────────┘     └─────────────────┘
         │                       │
         └───────────────────────┼───────────────────────┐
                                 │                       │
                         ┌───────▼────────┐     ┌────────▼──────┐
                         │ system_monitor │     │ submodules    │
                         │                │     │ (Raspberry Pi)│
                         │ - Health       │     │               │
                         │ - Monitoring   │     │ - Status      │
                         └────────────────┘     └───────────────┘
```

## Adding New Nodes

1. **Define in Graph Config**
   ```yaml
   robot:
     my_new_node:
       enabled: true
       package: my_package
       node: my_node
       namespace: /my_ns
       parameters:
         param1: value1
       topics:
         publish:
           - /my_ns/output
         subscribe:
           - /my_ns/input
   ```

2. **Add to Launch File**
   ```python
   my_node = Node(
       package='my_package',
       executable='my_node',
       name='my_node',
       namespace='my_ns',
       parameters=[{'param1': 'value1'}],
   )
   ```

3. **Add to Node Group**
   ```yaml
   groups:
     my_group:
       - my_new_node
   ```

4. **Rebuild**
   ```bash
   colcon build --packages-select isaac_robot
   ```

## Topic Remapping

Graph configs support topic remapping:

```yaml
remappings:
  - from: /camera/image
    to: /front_camera/image
```

## Best Practices

1. **Namespace Everything** - Use namespaces to avoid topic conflicts
2. **Document Topics** - List all publish/subscribe topics in config
3. **Use Groups** - Organize nodes into logical groups
4. **Parameterize** - Make thresholds and rates configurable
5. **Version Control** - Keep graph configs in git

## Verification

After launching, verify the graph:

```bash
# List all nodes
ros2 node list

# List all topics
ros2 topic list

# Check node info
ros2 node info /system/system_monitor

# Monitor system status
ros2 topic echo /system/status
```

## Future Enhancements

- Dynamic graph loading from YAML
- Graph visualization tool
- Runtime node enable/disable
- Graph validation
- Dependency checking
