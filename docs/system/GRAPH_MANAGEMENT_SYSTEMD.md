# Graph Management via Systemd

This guide covers the complete systemd integration for managing the robot's graph of nodes. All graph management should be done through the systemd daemon scripts.

## Overview

The Isaac robot system uses **systemd services** for full graph management:

1. **Systemd Service** (`isaac-robot.service`) - Manages the robot system lifecycle
2. **Graph Selection** - Configurable via file or environment variable
3. **Management Scripts** - Unified interface for graph operations
4. **Automatic Restart** - Service restarts on failure with exponential backoff

## Quick Start

### Start Robot System (via systemd)

```bash
# Use the unified graph management script (recommended)
./scripts/system/manage_graph.sh start bench_test

# Or use systemctl directly
sudo systemctl start isaac-robot.service
```

### Stop Robot System

```bash
./scripts/system/manage_graph.sh stop

# Or
sudo systemctl stop isaac-robot.service
```

### Check Status

```bash
./scripts/system/manage_graph.sh status

# Or
sudo systemctl status isaac-robot.service
```

## Graph Selection

### Available Graphs

- `minimal` - System monitor only
- `full` - All components (when available)
- `robot` - Default robot configuration
- `bench_test` - Complete bench setup (system_monitor + all hardware)

### Select Graph

```bash
# Select graph configuration
./scripts/system/manage_graph.sh select bench_test

# Restart to apply
./scripts/system/manage_graph.sh restart bench_test
```

This updates `config/robot/selected_graph.txt` which the systemd service reads on startup.

### Graph Selection Priority

1. **Environment Variable** (`ROBOT_GRAPH`) - Highest priority
2. **Config File** (`config/robot/selected_graph.txt`) - Persistent selection
3. **Default** (`minimal`) - Fallback

## Systemd Service Management

### Enable Service (Start on Boot)

```bash
sudo systemctl enable isaac-robot.service
```

### Start/Stop/Restart

```bash
# Start
sudo systemctl start isaac-robot.service

# Stop
sudo systemctl stop isaac-robot.service

# Restart
sudo systemctl restart isaac-robot.service
```

### View Logs

```bash
# Follow logs
sudo journalctl -u isaac-robot.service -f

# Last 100 lines
sudo journalctl -u isaac-robot.service -n 100

# Since boot
sudo journalctl -u isaac-robot.service -b
```

### Service Status

```bash
# Check if running
sudo systemctl is-active isaac-robot.service

# Check if enabled
sudo systemctl is-enabled isaac-robot.service

# Detailed status
sudo systemctl status isaac-robot.service
```

## Unified Management Script

The `manage_graph.sh` script provides a unified interface:

```bash
./scripts/system/manage_graph.sh <command> [options]
```

### Commands

- `start [graph]` - Start robot system with graph (via systemd)
- `stop` - Stop robot system
- `restart [graph]` - Restart robot system
- `status` - Show system status
- `select [graph]` - Select graph configuration
- `logs` - Show system logs
- `verify` - Verify data streams from all sensors

### Examples

```bash
# Start with bench_test graph
./scripts/system/manage_graph.sh start bench_test

# Check status
./scripts/system/manage_graph.sh status

# Select different graph
./scripts/system/manage_graph.sh select minimal

# Restart with new graph
./scripts/system/manage_graph.sh restart minimal

# Verify all sensors are publishing
./scripts/system/manage_graph.sh verify
```

## Service Configuration

### Override Graph via Environment Variable

Edit the service to override graph selection:

```bash
sudo systemctl edit isaac-robot.service
```

Add:
```ini
[Service]
Environment="ROBOT_GRAPH=bench_test"
```

Then reload and restart:
```bash
sudo systemctl daemon-reload
sudo systemctl restart isaac-robot.service
```

### Service File Location

The service file is at:
- Dev: `config/systemd/isaac-robot.service`
- Installed: `/etc/systemd/system/isaac-robot.service`

### Install Service

To install the service file:

```bash
cd ~/src/jetson-orin-nano
sudo cp config/systemd/isaac-robot.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable isaac-robot.service
```

## How It Works

### Startup Flow

1. **Systemd** starts `isaac-robot.service`
2. **Service** finds Isaac root directory
3. **Service** reads graph selection from `config/robot/selected_graph.txt` or `ROBOT_GRAPH` env var
4. **Service** calls `scripts/system/start_robot.sh`
5. **start_robot.sh** sources ROS 2 environment
6. **start_robot.sh** launches graph using `ros2 launch isaac_robot graph.launch.py`
7. **graph.launch.py** reads graph config and starts all enabled nodes

### Graph Configuration

The graph configuration (`config/robot/robot_graph.yaml`) defines:
- Which nodes to launch
- Node parameters
- Topic mappings
- Node groups (core, hardware, bench_test, etc.)

The launch file reads this config and creates ROS 2 `Node` actions for each enabled node in the selected group.

## Verification

### Verify Nodes Are Running

```bash
# Check systemd service
sudo systemctl status isaac-robot.service

# Check ROS 2 nodes
ros2 node list

# Check topics
ros2 topic list
```

### Verify Data Streams

```bash
# Use the verify command
./scripts/system/manage_graph.sh verify
```

This checks that all expected topics are publishing data.

## Troubleshooting

### Service Won't Start

1. **Check logs**:
   ```bash
   sudo journalctl -u isaac-robot.service -n 50
   ```

2. **Check graph selection**:
   ```bash
   cat config/robot/selected_graph.txt
   ```

3. **Check ROS 2 environment**:
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/ros2_ws/install/setup.bash
   ros2 --help
   ```

### Nodes Not Starting

1. **Check if packages are built**:
   ```bash
   ros2 pkg list | grep -E "(realsense|microphone|odrive|irobot)"
   ```

2. **Rebuild packages**:
   ```bash
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```

3. **Check graph config**:
   ```bash
   ./scripts/system/manage_graph.sh status
   ```

### Service Keeps Restarting

1. **Check service logs** for errors
2. **Check watchdog** - service has 60s watchdog timer
3. **Check restart limits** - service has restart limits configured
4. **Verify hardware** - ensure all hardware is connected

## Best Practices

1. **Always use `manage_graph.sh`** for graph operations
2. **Select graph before starting** - use `select` command
3. **Check status regularly** - use `status` command
4. **Monitor logs** - use `logs` command or `journalctl`
5. **Verify after changes** - use `verify` command

## Integration with Other Services

The robot service integrates with other systemd services:

- `isaac-system-monitor.service` - System health monitoring
- `isaac-health-check.service` - Periodic health checks
- `isaac-update.service` - System updates
- `isaac-restore.service` - System restoration

See [System Services](../deployment/SERVICES.md) for details.

## Next Steps

- See [Launching Guide](LAUNCHING.md) for manual launch instructions
- See [Graph Configuration](../robot/GRAPH_CONFIG.md) for graph config details
- See [Node Management](NODE_MANAGEMENT.md) for node-level management
