# Graph Configurations

The Isaac robot system uses a single graph configuration that includes all robot nodes and monitoring tools.

## Available Graph

### `robot` - Target/Production Graph

The main robot operation graph with all robot nodes:
- System monitor
- RealSense cameras (front and rear)
- USB microphone
- ODrive motor controller
- iRobot serial connection
- PHAT motor controller
- Foxglove Bridge (for remote visualization)
- VLA controller (when enabled)

**Use case**: Normal robot operation, production deployment, monitoring, and visualization

**Configuration**: `config/robot/robot_graph.yaml`

## Graph Selection

The robot graph is the default and only graph:

```bash
# Select robot graph (default)
./scripts/system/manage_graph.sh select robot

# View current selection
cat config/robot/selected_graph.txt
```

## Usage

### Start Robot Graph

```bash
# Select and start robot graph
./scripts/system/manage_graph.sh select robot
./scripts/system/manage_graph.sh start

# Or start directly (robot is default)
./scripts/system/manage_graph.sh start robot
```

## Default Graph

The default graph is `robot`. If no graph is selected, the system uses `robot`.

## Graph Configuration Files

- Robot graph: `config/robot/robot_graph.yaml`
- Selection file: `config/robot/selected_graph.txt`

## See Also

- [Graph Management via Systemd](GRAPH_MANAGEMENT_SYSTEMD.md) - Complete systemd integration guide
- [Systemd Integration](SYSTEMD_INTEGRATION.md) - Quick reference
