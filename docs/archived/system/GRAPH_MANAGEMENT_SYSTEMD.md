# Graph Management via Systemd

This guide covers systemd integration for managing the robot's graph of nodes. In production, the systemd user service is the single source of truth.

## Overview

The Isaac robot system uses **systemd services** for full graph management:

1. **Systemd Service** (`isaac-robot.service`) - Manages the robot system lifecycle
2. **Graph Selection** - Configurable via file or environment variable
3. **Launch Entry Point** - `scripts/system/start_robot.sh` used by systemd
4. **Automatic Restart** - Service restarts on failure with exponential backoff

## Quick Start

### Start Robot System (via systemd)

```bash
# Start via systemd user service (recommended)
systemctl --user start isaac-robot.service
```

### Stop Robot System

```bash
systemctl --user stop isaac-robot.service
```

### Check Status

```bash
systemctl --user status isaac-robot.service
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
echo "bench_test" > config/robot/selected_graph.txt

# Restart to apply
systemctl --user restart isaac-robot.service
```

This updates `config/robot/selected_graph.txt` which the systemd service reads on startup.

### Graph Selection Priority

1. **Environment Variable** (`ROBOT_GRAPH`) - Highest priority
2. **Config File** (`config/robot/selected_graph.txt`) - Persistent selection
3. **Default** (`minimal`) - Fallback

## Systemd Service Management

### Enable Service (Start on Boot)

```bash
systemctl --user enable isaac-robot.service
```

### Start/Stop/Restart

```bash
# Start
systemctl --user start isaac-robot.service

# Stop
systemctl --user stop isaac-robot.service

# Restart
systemctl --user restart isaac-robot.service
```

### View Logs

```bash
# Follow logs
journalctl --user -u isaac-robot.service -f

# Last 100 lines
journalctl --user -u isaac-robot.service -n 100

# Since boot
journalctl --user -u isaac-robot.service -b
```

### Service Status

```bash
# Check if running
systemctl --user is-active isaac-robot.service

# Check if enabled
systemctl --user is-enabled isaac-robot.service

# Detailed status
systemctl --user status isaac-robot.service
```

## Launching Without systemd (Development)

Use direct ROS 2 launch for development and debugging, but avoid running it alongside the systemd service.

```bash
# Stop systemd instance first (prevents duplicate graphs)
systemctl --user stop isaac-robot.service

# Launch the graph directly
ros2 launch isaac_robot composable_graph.launch.py \
  graph_config:=robot_graph.yaml \
  group:=sensor_pipeline \
  use_composable:=true
```

## Service Configuration

### Override Graph via Environment Variable

Edit the service to override graph selection:

```bash
systemctl --user edit isaac-robot.service
```

Add:
```ini
[Service]
Environment="ROBOT_GRAPH=bench_test"
```

Then reload and restart:
```bash
systemctl --user daemon-reload
systemctl --user restart isaac-robot.service
```

### Service File Location

The service file is at:
- Dev: `config/systemd/isaac-robot.service`
- Installed: `~/.config/systemd/user/isaac-robot.service`

### Install Service

To install the service file:

```bash
cd ~/src/jetson-orin-nano
mkdir -p ~/.config/systemd/user
cp config/systemd/isaac-robot.service ~/.config/systemd/user/
systemctl --user daemon-reload
systemctl --user enable isaac-robot.service
```

## How It Works

### Startup Flow

1. **Systemd** starts `isaac-robot.service`
2. **Service** finds Isaac root directory
3. **Service** reads graph selection from `config/robot/selected_graph.txt` or `ROBOT_GRAPH` env var
4. **Service** calls `scripts/system/start_robot.sh`
5. **start_robot.sh** sources ROS 2 environment
6. **start_robot.sh** launches graph using `ros2 launch isaac_robot composable_graph.launch.py`
7. **composable_graph.launch.py** reads graph config and starts all enabled nodes

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
systemctl --user status isaac-robot.service

# Check ROS 2 nodes
ros2 node list

# Check topics
ros2 topic list
```

### Verify Data Streams

```bash
# Verify expected topics
ros2 topic list
```

This checks that all expected topics are publishing data.

## Troubleshooting

### Service Won't Start

1. **Check logs**:
   ```bash
   journalctl --user -u isaac-robot.service -n 50
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
   ros2 pkg list | grep -E "(realsense|microphone|phat|irobot)"
   ```

2. **Rebuild packages**:
   ```bash
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```

3. **Check service status**:
   ```bash
   systemctl --user status isaac-robot.service
   ```

### Service Keeps Restarting

1. **Check service logs** for errors
2. **Check watchdog** - service has 60s watchdog timer
3. **Check restart limits** - service has restart limits configured
4. **Verify hardware** - ensure all hardware is connected

## Best Practices

1. **Use systemd for production** - avoid manual `ros2 launch`
2. **Avoid multiple instances** - never run systemd and manual launch at the same time
3. **Monitor logs** - use `journalctl --user -u isaac-robot.service -f`
4. **Verify after changes** - use `ros2 topic list` / `ros2 node list`

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
