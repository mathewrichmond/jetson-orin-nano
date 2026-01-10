# Launching the Robot System

**IMPORTANT**: Always use centralized graph management. Do NOT launch nodes ad hoc with `ros2 launch` or `ros2 run`. Use the graph management script instead.

## Centralized Graph Management (Required)

The Isaac robot system uses **centralized graph management** through the `manage_graph.sh` script. This is the ONLY way to launch nodes in production.

### Quick Start

```bash
cd ~/src/jetson-orin-nano

# Start robot graph (production)
./scripts/system/manage_graph.sh start robot

# Start monitor graph (viewing/logging)
./scripts/system/manage_graph.sh start monitor

# Check status
./scripts/system/manage_graph.sh status

# Verify data streams
./scripts/system/manage_graph.sh verify
```

## Why Centralized Management?

- **Consistent configuration** - All nodes use the same graph config
- **Proper namespaces** - Nodes are configured correctly
- **Systemd integration** - Works with service management
- **Single source of truth** - Graph config defines everything
- **Easy switching** - Switch between robot/monitor graphs easily

## Available Graphs

- `robot` - Target/production graph (all robot nodes)
- `monitor` - Viewer/logger graph (monitoring tools)

See [Graph Configurations](GRAPH_CONFIGURATIONS.md) for details.

## Graph Management Commands

```bash
# Select graph
./scripts/system/manage_graph.sh select robot
./scripts/system/manage_graph.sh select monitor

# Start graph
./scripts/system/manage_graph.sh start [robot|monitor]

# Stop graph
./scripts/system/manage_graph.sh stop

# Restart graph
./scripts/system/manage_graph.sh restart [robot|monitor]

# Check status
./scripts/system/manage_graph.sh status

# Verify data streams
./scripts/system/manage_graph.sh verify

# View logs
./scripts/system/manage_graph.sh logs
```

## Launch File Structure

The launch system uses:

1. **Graph Configuration** (`config/robot/robot_graph.yaml`) - Defines all nodes, parameters, and groups
2. **Launch File** (`src/isaac_robot/launch/graph.launch.py`) - Standard ROS 2 launch file that reads the graph config
3. **Node Entry Points** - Each node has an entry point defined in its `setup.py`

## How It Works

1. **Graph Config** defines which nodes to launch and their configuration
2. **Launch File** reads the config and creates `Node` actions for each enabled node
3. **ROS 2 Launch** finds executables via entry points and launches them
4. **Nodes** start and begin publishing/subscribing to topics

## Available Groups

From `config/robot/robot_graph.yaml`:

- `core` - Essential nodes (system_monitor)
- `hardware` - All hardware drivers (cameras, microphone, motors, iRobot)
- `bench_test` - Complete bench setup (system_monitor + all hardware)
- `control` - Control nodes (VLA controller - when enabled)
- `all` - All enabled nodes

## Verification

After launching, verify nodes are running:

```bash
# List running nodes
ros2 node list

# List active topics
ros2 topic list

# Check specific node
ros2 node info /hardware/realsense_camera

# Monitor a topic
ros2 topic echo /realsense/status
```

## Troubleshooting

### Nodes Not Starting

1. **Check executables exist**:
   ```bash
   ros2 pkg executables <package_name>
   ```

2. **Rebuild packages**:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select <package_name>
   source install/setup.bash
   ```

3. **Check launch file output**:
   ```bash
   ros2 launch isaac_robot graph.launch.py group:=bench_test
   # Look for error messages in the output
   ```

### Topics Not Publishing

1. **Verify nodes are running**:
   ```bash
   ros2 node list
   ```

2. **Check node status**:
   ```bash
   ros2 node info /namespace/node_name
   ```

3. **Check hardware connections** (for hardware nodes)

### Launch File Not Found

The launch file should be installed with the `isaac_robot` package. If not found:

```bash
cd ~/ros2_ws
colcon build --packages-select isaac_robot
source install/setup.bash
```

## ⚠️ Do NOT Launch Nodes Ad Hoc

**Never use these commands in production:**
- ❌ `ros2 launch isaac_robot graph.launch.py` (use manage_graph.sh instead)
- ❌ `ros2 run <package> <node>` (nodes should be in graph config)
- ❌ `ros2 launch <package> <launch_file>` (use graph management)

**Why?**
- Nodes won't have correct namespaces
- Parameters won't match production config
- Service management won't work
- Graph state becomes inconsistent

## Debugging Individual Nodes

If you need to test a single node for development:

1. **Add node to graph config** (`config/robot/robot_graph.yaml`)
2. **Enable it** in the graph
3. **Use graph management** to start it: `./scripts/system/manage_graph.sh start robot`

Only use `ros2 run` for initial node development/testing before adding to graph.

## Next Steps

- See [Graph Configuration](../robot/GRAPH_CONFIG.md) for details on configuring nodes
- See [Node Management](NODE_MANAGEMENT.md) for advanced node management
- See [Hardware Setup](../hardware/HARDWARE_SETUP.md) for hardware-specific launch instructions
