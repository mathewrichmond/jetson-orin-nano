# Node Management Quick Reference

Quick reference for managing ROS 2 nodes in the Isaac robot system.

## Centralized Graph Management (Required)

**ALWAYS use graph management. Never launch nodes ad hoc.**

### Start Nodes

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

### ⚠️ Do NOT Use These

- ❌ `ros2 launch` - Use `manage_graph.sh` instead
- ❌ `ros2 run` - Nodes should be in graph config
- ❌ Individual node launch - Use graph management

### Available Graphs

- `robot` - Target/production graph (all robot nodes)
- `monitor` - Viewer/logger graph (monitoring tools)

## Graph Management Commands

```bash
# Select graph
./scripts/system/manage_graph.sh select [robot|monitor]

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
3. **Add to graph config** (`config/robot/robot_graph.yaml` or `config/robot/monitor_graph.yaml`)
4. Rebuild: `colcon build --packages-select package_name`
5. **Use graph management** to start: `./scripts/system/manage_graph.sh start robot`

**Never use `ros2 run` in production** - always add to graph config and use graph management.
5. Test: `ros2 run package_name node_name`

See [Node Management Guide](NODE_MANAGEMENT.md) for details.
