# Systemd Integration for Graph Management

## Overview

Full graph management is done through the systemd user service. This provides:
- Automatic startup on boot
- Service lifecycle management
- Automatic restart on failure
- Centralized logging
- Unified management interface

## Architecture

```
systemd (isaac-robot.service)
    ↓
start_robot.sh (reads graph selection)
    ↓
ros2 launch isaac_robot composable_graph.launch.py
    ↓
composable_graph.launch.py (reads robot_graph.yaml)
    ↓
ROS 2 Nodes (system_monitor, cameras, etc.)
```

## Primary Interface: systemd user service

```bash
systemctl --user start isaac-robot.service
systemctl --user stop isaac-robot.service
systemctl --user restart isaac-robot.service
systemctl --user status isaac-robot.service
journalctl --user -u isaac-robot.service -f
```

## Setup

### 1. Install Service

```bash
cd ~/src/jetson-orin-nano
mkdir -p ~/.config/systemd/user
cp config/systemd/isaac-robot.service ~/.config/systemd/user/
systemctl --user daemon-reload
systemctl --user enable isaac-robot.service
```

### 2. Select Graph

```bash
echo "bench_test" > config/robot/selected_graph.txt
```

### 3. Start Service

```bash
systemctl --user start isaac-robot.service
```

## Graph Selection

Graphs are selected via `config/robot/selected_graph.txt`:

```bash
# Select graph
echo "bench_test" > config/robot/selected_graph.txt

# View current selection
cat config/robot/selected_graph.txt
```

Available graphs:
- `minimal` - System monitor only
- `robot` - Default robot config
- `bench_test` - All hardware (cameras, microphone, motors, iRobot)
- `full` - Complete system

## Service Management

### Enable/Disable Auto-Start

```bash
# Enable (start on login)
systemctl --user enable isaac-robot.service

# Disable
systemctl --user disable isaac-robot.service
```

### Start/Stop/Restart

```bash
systemctl --user start isaac-robot.service
systemctl --user stop isaac-robot.service
systemctl --user restart isaac-robot.service
```

### Status and Logs

```bash
# Status
systemctl --user status isaac-robot.service

# Logs
journalctl --user -u isaac-robot.service -f
```

## Service Configuration

### Override Graph via Environment

Edit service:
```bash
sudo systemctl edit isaac-robot.service
```

Add:
```ini
[Service]
Environment="ROBOT_GRAPH=bench_test"
```

Reload:
```bash
systemctl --user daemon-reload
systemctl --user restart isaac-robot.service
```

### Service File Location

- Source: `config/systemd/isaac-robot.service`
- Installed: `~/.config/systemd/user/isaac-robot.service`

## Verification

### Check Service Status

```bash
systemctl --user status isaac-robot.service
```

### Check Nodes Running

```bash
ros2 node list
```

### Verify Data Streams

```bash
ros2 topic list
```

## Troubleshooting

### Service Failing to Start

1. Check logs:
   ```bash
   journalctl --user -u isaac-robot.service -n 100
   ```

2. Check graph selection:
   ```bash
   cat config/robot/selected_graph.txt
   ```

3. Test launch manually:
   ```bash
   ./scripts/system/start_robot.sh
   ```

### Service Restarting Too Often

The service has restart limits. If it keeps failing:
1. Check logs for root cause
2. Verify hardware connections
3. Check ROS 2 packages are built
4. Temporarily disable service: `systemctl --user stop isaac-robot.service`

## Best Practices

1. **Use systemd for production** - avoid manual `ros2 launch`
2. **Select graph before starting** - ensures correct configuration
3. **Enable service for production** - automatic startup on login
4. **Monitor logs regularly** - catch issues early
5. **Verify after changes** - use `ros2 topic list` / `ros2 node list`

## Integration Points

The systemd service integrates with:
- `scripts/system/start_robot.sh` - Launches graph
- `scripts/utils/get_graph.sh` - Reads graph selection
- `scripts/utils/select_graph.sh` - Sets graph selection
- `config/robot/robot_graph.yaml` - Graph configuration

## See Also

- [Graph Management via Systemd](GRAPH_MANAGEMENT_SYSTEMD.md) - Detailed guide
- [Graph Management](GRAPH_MANAGEMENT.md) - General concepts
- [Launching Guide](LAUNCHING.md) - Manual launch (for development)
