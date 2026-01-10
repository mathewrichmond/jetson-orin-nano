# Graph Configurations

The Isaac robot system uses two graph configurations:

## Available Graphs

### 1. `robot` - Target/Production Graph

The main robot operation graph with all robot nodes:
- System monitor
- RealSense cameras (front and rear)
- USB microphone
- ODrive motor controller
- iRobot serial connection
- VLA controller (when enabled)

**Use case**: Normal robot operation, production deployment

**Configuration**: `config/robot/robot_graph.yaml`

### 2. `monitor` - Viewer/Logger Graph

Monitoring and visualization tools:
- System monitor
- Foxglove Bridge (for remote visualization)

**Use case**: Monitoring, debugging, visualization, logging

**Configuration**: `config/robot/monitor_graph.yaml`

## Graph Selection

Only one graph runs at a time. Select which graph to use:

```bash
# Select robot graph
./scripts/system/manage_graph.sh select robot

# Select monitor graph
./scripts/system/manage_graph.sh select monitor

# View current selection
cat config/robot/selected_graph.txt
```

## Usage

### Start Robot Graph

```bash
# Select and start robot graph
./scripts/system/manage_graph.sh select robot
./scripts/system/manage_graph.sh start

# Or start directly
./scripts/system/manage_graph.sh start robot
```

### Start Monitor Graph

```bash
# Select and start monitor graph
./scripts/system/manage_graph.sh select monitor
./scripts/system/manage_graph.sh start

# Or start directly
./scripts/system/manage_graph.sh start monitor
```

## Switching Between Graphs

To switch from one graph to another:

```bash
# Stop current graph
./scripts/system/manage_graph.sh stop

# Select new graph
./scripts/system/manage_graph.sh select monitor

# Start new graph
./scripts/system/manage_graph.sh start
```

Or restart with new graph:

```bash
./scripts/system/manage_graph.sh restart monitor
```

## Default Graph

The default graph is `robot`. If no graph is selected, the system uses `robot`.

## Graph Configuration Files

- Robot graph: `config/robot/robot_graph.yaml`
- Monitor graph: `config/robot/monitor_graph.yaml`
- Selection file: `config/robot/selected_graph.txt`

## See Also

- [Graph Management via Systemd](GRAPH_MANAGEMENT_SYSTEMD.md) - Complete systemd integration guide
- [Systemd Integration](SYSTEMD_INTEGRATION.md) - Quick reference
