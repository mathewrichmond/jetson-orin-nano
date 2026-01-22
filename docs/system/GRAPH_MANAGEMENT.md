# Graph Management

**IMPORTANT**: Full graph management should be done through the systemd user service. See [Graph Management via Systemd](GRAPH_MANAGEMENT_SYSTEMD.md) for the recommended production approach.

This document covers graph management concepts and manual operations. Guide

## Overview

The Isaac robot system uses ROS 2 launch integrated with systemd for runtime management. In production, use the systemd user service.

## Primary Interfaces

- **Production**: `systemctl --user start|stop|restart isaac-robot.service`
- **Development**: `ros2 launch isaac_robot composable_graph.launch.py ...`

## Commands

### Start Robot System

Start the robot system with a specific graph:

```bash
# Start with selected graph (from config)
systemctl --user start isaac-robot.service
```

### Stop Robot System

```bash
systemctl --user stop isaac-robot.service
```

### Restart Robot System

```bash
systemctl --user restart isaac-robot.service
```

### Select Graph Configuration

Select which graph to use (applies on next start):

```bash
echo "bench_test" > config/robot/selected_graph.txt
systemctl --user restart isaac-robot.service
```

### Check Status

```bash
systemctl --user status isaac-robot.service
```

Shows:
- Systemd service status
- Current graph selection
- Running ROS 2 nodes
- Active topics

### View Logs

```bash
journalctl --user -u isaac-robot.service -f
```

### Verify Data Streams

Verify all sensor data streams are publishing:

```bash
ros2 topic list
```

## Available Graphs

- **`minimal`** - System monitor only
- **`full`** - Complete system with all components
- **`robot`** - Default robot configuration
- **`bench_test`** - Bench test configuration (all hardware + visualization)

## Systemd Integration

The graph runtime is managed through systemd:

```bash
# Enable auto-start on login
systemctl --user enable isaac-robot.service

# Start service
systemctl --user start isaac-robot.service

# Check status
systemctl --user status isaac-robot.service

# View logs
journalctl --user -u isaac-robot.service -f
```

The service automatically uses the selected graph from `config/robot/selected_graph.txt`.

## Graph Selection Priority

1. **Environment variable** - `ROBOT_GRAPH=bench_test`
2. **Config file** - `config/robot/selected_graph.txt`
3. **Default** - `minimal`

## Examples

### Bench Test Setup

```bash
# Select bench test graph
echo "bench_test" > config/robot/selected_graph.txt

# Start system
systemctl --user start isaac-robot.service

# Verify all sensors are streaming
ros2 topic list

# View logs
journalctl --user -u isaac-robot.service -f
```

### Production Deployment

```bash
# Select full graph
echo "full" > config/robot/selected_graph.txt

# Enable and start service
systemctl --user enable isaac-robot.service
systemctl --user start isaac-robot.service

# Check status
systemctl --user status isaac-robot.service
```

### Development Testing

```bash
# Select minimal graph for testing
echo "minimal" > config/robot/selected_graph.txt

# Start via systemd
systemctl --user start isaac-robot.service

# Test changes, then restart
systemctl --user restart isaac-robot.service
```

## Integration with Hardware Setup

Hardware setup and verification is separate:

```bash
# Install hardware components
./scripts/hardware/setup_hardware.sh install all

# Verify hardware is connected
./scripts/hardware/setup_hardware.sh verify

# Then start graph
systemctl --user start isaac-robot.service
```

## Troubleshooting

### Service Won't Start

```bash
# Check service status
systemctl --user status isaac-robot.service

# Check logs
journalctl --user -u isaac-robot.service -n 50

# Check graph selection
cat config/robot/selected_graph.txt
```

### No Data Streams

```bash
# Verify hardware is connected
./scripts/hardware/setup_hardware.sh verify

# Check ROS 2 topics
ros2 topic list
```

### Graph Not Found

```bash
# List available graphs
ls config/robot/*_graph.yaml

# Select valid graph
echo "<graph_name>" > config/robot/selected_graph.txt
```

## See Also

- [Hardware Setup](../hardware/HARDWARE_SETUP.md) - Hardware installation
- [Node Management](NODE_MANAGEMENT.md) - Detailed node management
- [Graph Configuration](../robot/GRAPH_CONFIG.md) - Graph configuration details
