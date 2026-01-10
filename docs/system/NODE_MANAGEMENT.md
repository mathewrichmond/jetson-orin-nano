# Node Management Guide

This guide covers the consistent node management system for the Isaac robot, including entry points, graph configuration, and systemd integration.

## Overview

The Isaac robot system uses a **graph-based node management** approach:

1. **Graph Configuration** - YAML files define all nodes and their configuration
2. **Graph Manager** - Launches nodes from graph configuration
3. **Entry Points** - All nodes have consistent ROS 2 entry points
4. **Systemd Integration** - Nodes can run as system services
5. **Management Tools** - Consistent scripts for node lifecycle management

## Entry Points

All Python nodes have proper entry points defined in `setup.py`:

```python
entry_points={
    'console_scripts': [
        'node_name = package.module:main',
    ],
}
```

Entry point scripts are automatically created in `lib/<package>/` during build, ensuring `ros2 run` works correctly even with symlink installs.

### Current Nodes

- `system_monitor_node` - System monitoring
- `realsense_camera_node` - RealSense cameras
- `hello_world_node` - Example/test node

## Graph Configuration

Graph configurations are YAML files in `config/robot/`:

- `robot_graph.yaml` - Default configuration
- `minimal_graph.yaml` - Minimal system (system monitor only)
- `full_graph.yaml` - Full system with all components

### Node Definition

```yaml
robot:
  node_name:
    enabled: true
    package: package_name
    node: node_executable
    namespace: /namespace
    parameters:
      param1: value1
      param2: value2
    topics:
      publish:
        - /topic1
        - /topic2
      subscribe:
        - /topic3
```

### Node Groups

Groups allow launching subsets of nodes:

```yaml
groups:
  core:
    - system_monitor
  hardware:
    - realsense_camera
  all:
    - system_monitor
    - realsense_camera
```

## Node Management Tools

### manage_nodes.sh

Unified script for managing node runtimes:

```bash
# Start nodes from graph
./scripts/system/manage_nodes.sh start [graph] [group]

# Examples
./scripts/system/manage_nodes.sh start robot_graph.yaml core
./scripts/system/manage_nodes.sh start full_graph.yaml all

# Stop all nodes
./scripts/system/manage_nodes.sh stop

# Show status
./scripts/system/manage_nodes.sh status

# List available nodes
./scripts/system/manage_nodes.sh list [graph]

# Restart nodes
./scripts/system/manage_nodes.sh restart [graph] [group]

# View logs
./scripts/system/manage_nodes.sh logs [node_name]
```

### start_robot.sh

High-level robot startup script (used by systemd):

```bash
./scripts/system/start_robot.sh
```

Uses `ROBOT_GRAPH` environment variable or defaults to `robot_graph.yaml`.

## Launching Nodes

**IMPORTANT**: Always use centralized graph management. Do NOT launch nodes ad hoc.

### Centralized Graph Management (Required)

```bash
# Start robot graph (production)
./scripts/system/manage_graph.sh start robot

# Start monitor graph (viewing/logging)
./scripts/system/manage_graph.sh start monitor

# Check status
./scripts/system/manage_graph.sh status

# Verify data streams
./scripts/system/manage_graph.sh verify
```

### ⚠️ Do NOT Use These Commands

**Never use these in production:**
- ❌ `ros2 launch isaac_robot graph.launch.py` - Use `manage_graph.sh` instead
- ❌ `ros2 run <package> <node>` - Nodes should be in graph config
- ❌ `ros2 launch <package> <launch_file>` - Use graph management

**Why?**
- Nodes won't have correct namespaces
- Parameters won't match production config
- Service management won't work
- Graph state becomes inconsistent

### Development Testing

For initial node development (before adding to graph):
- Use `ros2 run` only for testing the node itself
- Once working, add to graph config (`config/robot/robot_graph.yaml`)
- Then use graph management to launch: `./scripts/system/manage_graph.sh start robot`

## Systemd Integration

### Service File

The `isaac-robot.service` systemd unit manages the robot system:

```bash
# Enable service (starts on boot)
sudo systemctl enable isaac-robot.service

# Start service
sudo systemctl start isaac-robot.service

# Stop service
sudo systemctl stop isaac-robot.service

# Check status
sudo systemctl status isaac-robot.service

# View logs
sudo journalctl -u isaac-robot.service -f
```

### Service Configuration

Edit service file to change graph:

```bash
sudo systemctl edit isaac-robot.service
```

Add:
```ini
[Service]
Environment="ROBOT_GRAPH=minimal"
```

### Service Management

```bash
# Restart service
sudo systemctl restart isaac-robot.service

# Reload configuration
sudo systemctl daemon-reload
sudo systemctl restart isaac-robot.service
```

## Adding New Nodes

### 1. Create Node Package

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_node_package
```

### 2. Add Entry Point

In `setup.py`:

```python
entry_points={
    'console_scripts': [
        'my_node = my_node_package.my_node:main',
    ],
}
```

### 3. Add Entry Point Script (CMakeLists.txt)

For Python packages, add to `CMakeLists.txt`:

```cmake
install(CODE "
  file(MAKE_DIRECTORY \"${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}\")
  file(WRITE \"${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/my_node\"
    \"#!/usr/bin/env python3\\n\"
    \"from my_node_package.my_node import main\\n\"
    \"if __name__ == '__main__':\\n\"
    \"    main()\\n\"
  )
  file(CHMOD \"${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/my_node\"
    PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
                GROUP_READ GROUP_EXECUTE
                WORLD_READ WORLD_EXECUTE
  )
")
```

### 4. Add to Graph Config

In `config/robot/robot_graph.yaml`:

```yaml
robot:
  my_node:
    enabled: true
    package: my_node_package
    node: my_node
    namespace: /my_namespace
    parameters:
      param1: value1
    topics:
      publish:
        - /my_namespace/output
      subscribe:
        - /my_namespace/input

groups:
  my_group:
    - my_node
  all:
    - system_monitor
    - my_node
```

### 5. Build and Test

```bash
cd ~/ros2_ws
colcon build --packages-select my_node_package isaac_robot
source install/setup.bash

# Test node directly
ros2 run my_node_package my_node

# Test via graph
ros2 launch isaac_robot graph.launch.py group:=my_group
```

## Verification

### Check Entry Points

```bash
# List all available nodes
ros2 pkg executables

# Check specific package
ros2 pkg executables my_package

# Test node
ros2 run my_package my_node --help
```

### Check Graph

```bash
# List nodes in graph
./scripts/system/manage_nodes.sh list robot_graph.yaml

# Check node status
./scripts/system/manage_nodes.sh status
```

### Check Running Nodes

```bash
# List running nodes
ros2 node list

# Get node info
ros2 node info /namespace/node_name

# Check topics
ros2 topic list
```

## Best Practices

1. **Always use entry points** - Never run Python nodes directly
2. **Define in graph config** - All nodes should be in graph YAML
3. **Use namespaces** - Avoid topic conflicts
4. **Document topics** - List all publish/subscribe topics
5. **Use groups** - Organize nodes logically
6. **Test entry points** - Verify `ros2 run` works before adding to graph
7. **Use systemd for production** - Manual launch for development

## Troubleshooting

### Entry Point Not Found

```bash
# Rebuild package
cd ~/ros2_ws
colcon build --packages-select package_name
source install/setup.bash

# Check entry point exists
ls -la ~/ros2_ws/install/package_name/lib/package_name/
```

### Node Not Starting from Graph

1. Check node is enabled in graph config
2. Verify entry point works: `ros2 run package node`
3. Check graph config syntax: `./scripts/system/manage_nodes.sh list`
4. Check launch logs for errors

### Systemd Service Issues

```bash
# Check service status
sudo systemctl status isaac-robot.service

# View logs
sudo journalctl -u isaac-robot.service -n 100

# Check environment
sudo systemctl show isaac-robot.service
```

## See Also

- [Graph Configuration Guide](../robot/GRAPH_CONFIG.md)
- [Systemd Service Files](../../config/systemd/)
- [Development Workflow](../development/WORKFLOW.md)
