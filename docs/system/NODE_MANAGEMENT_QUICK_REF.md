# Node Management Quick Reference

Quick reference for managing ROS 2 nodes in the Isaac robot system.

## Primary Management (Required)

**Use systemd for production, ROS 2 launch for development. Avoid multiple instances.**

### Start Nodes

```bash
# Start robot graph (production)
systemctl --user start isaac-robot.service

# Check status
systemctl --user status isaac-robot.service

# Verify data streams
ros2 topic list
```

### ⚠️ Avoid Multiple Instances

- ❌ Running systemd and manual `ros2 launch` concurrently
- ❌ Multiple `ros2 launch` instances of the same graph

### Available Graphs

- `robot` - Target/production graph (all robot nodes)
- `monitor` - Viewer/logger graph (monitoring tools)

## Graph Management Commands (systemd)

```bash
# Select graph
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

## Systemd Service

```bash
# Enable (start on login)
systemctl --user enable isaac-robot.service

# Start
systemctl --user start isaac-robot.service

# Stop
systemctl --user stop isaac-robot.service

# Status
systemctl --user status isaac-robot.service

# Logs
journalctl --user -u isaac-robot.service -f
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
3. **Add to graph config** (`config/robot/robot_graph.yaml` or `config/robot/monitor_graph.yaml`)
4. Rebuild: `colcon build --packages-select package_name`
5. **Use systemd** to start: `systemctl --user start isaac-robot.service`

**Never use `ros2 run` in production** - always add to graph config and use graph management.
5. Test: `ros2 run package_name node_name`

See [Node Management Guide](NODE_MANAGEMENT.md) for details.
