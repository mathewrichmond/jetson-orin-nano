# Systemd Integration for Graph Management

## Overview

Full graph management is done through systemd daemon scripts. This provides:
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
ros2 launch isaac_robot graph.launch.py
    ↓
graph.launch.py (reads robot_graph.yaml)
    ↓
ROS 2 Nodes (system_monitor, cameras, etc.)
```

## Primary Interface: manage_graph.sh

**All graph management should use this script:**

```bash
./scripts/system/manage_graph.sh <command> [options]
```

### Commands

- `start [graph]` - Start robot via systemd
- `stop` - Stop robot via systemd
- `restart [graph]` - Restart robot via systemd
- `status` - Show service and node status
- `select [graph]` - Select graph configuration
- `logs` - View service logs
- `verify` - Verify all data streams

## Setup

### 1. Install Service

```bash
cd ~/src/jetson-orin-nano
sudo cp config/systemd/isaac-robot.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable isaac-robot.service
```

### 2. Select Graph

```bash
./scripts/system/manage_graph.sh select bench_test
```

### 3. Start Service

```bash
./scripts/system/manage_graph.sh start
# Or
sudo systemctl start isaac-robot.service
```

## Graph Selection

Graphs are selected via `config/robot/selected_graph.txt`:

```bash
# Select graph
./scripts/system/manage_graph.sh select bench_test

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
# Enable (start on boot)
sudo systemctl enable isaac-robot.service

# Disable
sudo systemctl disable isaac-robot.service
```

### Start/Stop/Restart

```bash
# Via manage_graph.sh (recommended)
./scripts/system/manage_graph.sh start
./scripts/system/manage_graph.sh stop
./scripts/system/manage_graph.sh restart

# Via systemctl
sudo systemctl start isaac-robot.service
sudo systemctl stop isaac-robot.service
sudo systemctl restart isaac-robot.service
```

### Status and Logs

```bash
# Status
./scripts/system/manage_graph.sh status

# Logs
./scripts/system/manage_graph.sh logs
# Or
sudo journalctl -u isaac-robot.service -f
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
sudo systemctl daemon-reload
sudo systemctl restart isaac-robot.service
```

### Service File Location

- Source: `config/systemd/isaac-robot.service`
- Installed: `/etc/systemd/system/isaac-robot.service`

## Verification

### Check Service Status

```bash
sudo systemctl status isaac-robot.service
```

### Check Nodes Running

```bash
ros2 node list
```

### Verify Data Streams

```bash
./scripts/system/manage_graph.sh verify
```

## Troubleshooting

### Service Failing to Start

1. Check logs:
   ```bash
   sudo journalctl -u isaac-robot.service -n 100
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
4. Temporarily disable service: `sudo systemctl stop isaac-robot.service`

## Best Practices

1. **Always use `manage_graph.sh`** for graph operations
2. **Select graph before starting** - ensures correct configuration
3. **Enable service for production** - automatic startup on boot
4. **Monitor logs regularly** - catch issues early
5. **Verify after changes** - use `verify` command

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
