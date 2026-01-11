# Node Launching Policy

## ⚠️ CRITICAL: Always Use Centralized Graph Management

**DO NOT launch nodes ad hoc.** Always use the centralized graph management system.

## Required Method

**ALWAYS use:**
```bash
./scripts/system/manage_graph.sh start [robot|monitor]
```

**NEVER use:**
- ❌ `ros2 launch isaac_robot graph.launch.py`
- ❌ `ros2 run <package> <node>`
- ❌ `ros2 launch <package> <launch_file>`
- ❌ Any direct ROS 2 launch/run commands

## Why Centralized Management?

1. **Consistent Configuration** - All nodes use the same graph config
2. **Proper Namespaces** - Nodes are configured with correct namespaces
3. **Systemd Integration** - Works with service management for boot startup
4. **Single Source of Truth** - Graph config (`config/robot/robot_graph.yaml`) defines everything
6. **State Management** - Service tracks state, handles restarts, provides logs

## Available Graph

- **`robot`** - Target/production graph (all robot nodes: cameras, motors, sensors, monitoring tools, etc.)

## Graph Management Commands

```bash
# Select graph (robot is default)
./scripts/system/manage_graph.sh select robot

# Start graph
./scripts/system/manage_graph.sh start robot

# Stop graph
./scripts/system/manage_graph.sh stop

# Restart graph
./scripts/system/manage_graph.sh restart robot

# Check status
./scripts/system/manage_graph.sh status

# Verify data streams
./scripts/system/manage_graph.sh verify

# View logs
./scripts/system/manage_graph.sh logs
```

## Adding Nodes

When adding a new node:

1. **Add to graph config** (`config/robot/robot_graph.yaml`)
2. **Enable the node** in the graph config
3. **Use graph management** to start: `./scripts/system/manage_graph.sh start robot`

**Do NOT** test with `ros2 run` in production - only use for initial development before adding to graph.

## Development Exception

**Only exception**: Initial node development/testing before adding to graph:
- Use `ros2 run <package> <node>` to test the node itself
- Once working, add to graph config
- Then use graph management: `./scripts/system/manage_graph.sh start robot`

## Documentation

All documentation should:
- Show `manage_graph.sh` as the primary method
- Only mention `ros2 run` for initial development
- Never show `ros2 launch` as a production method
- Emphasize graph management in all examples

## See Also

- [Graph Management via Systemd](GRAPH_MANAGEMENT_SYSTEMD.md) - Complete guide
- [Graph Configurations](GRAPH_CONFIGURATIONS.md) - Graph details
- [Systemd Integration](SYSTEMD_INTEGRATION.md) - Service management
