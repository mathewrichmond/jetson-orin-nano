# Node Launching Policy

## ⚠️ CRITICAL: Use Systemd or ROS 2 Launch

**Do not launch nodes ad hoc.** Use systemd in production and ROS 2 launch for development.

## Required Methods

**Production (systemd user service):**
```bash
systemctl --user start isaac-robot.service
```

**Development (direct launch):**
```bash
systemctl --user stop isaac-robot.service
ros2 launch isaac_robot composable_graph.launch.py \
  graph_config:=robot_graph.yaml \
  group:=sensor_pipeline \
  use_composable:=true
```

**Avoid multiple instances:**
- Do not run systemd and a manual `ros2 launch` concurrently.
- Do not run multiple `ros2 launch` instances of the same graph.

## Why Centralized Management?

1. **Consistent Configuration** - All nodes use the same graph config
2. **Proper Namespaces** - Nodes are configured with correct namespaces
3. **Systemd Integration** - Works with service management for boot startup
4. **Single Source of Truth** - Graph config (`config/robot/robot_graph.yaml`) defines everything
6. **State Management** - Service tracks state, handles restarts, provides logs

## Available Graph

- **`robot`** - Target/production graph (all robot nodes: cameras, motors, sensors, monitoring tools, etc.)

## Graph Management Commands (systemd)

```bash
# Select graph (robot is default)
echo "robot" > config/robot/selected_graph.txt

# Start graph
systemctl --user start isaac-robot.service

# Stop graph
systemctl --user stop isaac-robot.service

# Restart graph
systemctl --user restart isaac-robot.service

# Check status
systemctl --user status isaac-robot.service

# Verify data streams
ros2 topic list

# View logs
journalctl --user -u isaac-robot.service -f
```

## Adding Nodes

When adding a new node:

1. **Add to graph config** (`config/robot/robot_graph.yaml`)
2. **Enable the node** in the graph config
3. **Use systemd** to start: `systemctl --user start isaac-robot.service`

**Do NOT** test with `ros2 run` in production - only use for initial development before adding to graph.

## Development Exception

**Only exception**: Initial node development/testing before adding to graph:
- Use `ros2 run <package> <node>` to test the node itself
- Once working, add to graph config
- Then use systemd: `systemctl --user start isaac-robot.service`

## Documentation

All documentation should:
- Show systemd as the primary method for production
- Only mention `ros2 run` for initial development
- Avoid showing ad hoc `ros2 launch` in production contexts
- Emphasize single-instance management in all examples

## See Also

- [Graph Management via Systemd](GRAPH_MANAGEMENT_SYSTEMD.md) - Complete guide
- [Graph Configurations](GRAPH_CONFIGURATIONS.md) - Graph details
- [Systemd Integration](SYSTEMD_INTEGRATION.md) - Service management
