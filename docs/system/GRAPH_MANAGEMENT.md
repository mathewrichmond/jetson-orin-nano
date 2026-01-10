# Graph Management

**IMPORTANT**: Full graph management should be done through systemd daemon scripts. See [Graph Management via Systemd](GRAPH_MANAGEMENT_SYSTEMD.md) for the recommended production approach.

This document covers graph management concepts and manual operations. Guide

## Overview

The Isaac robot system uses a unified graph management system that integrates with systemd for runtime management. All graph operations go through the unified management script.

## Unified Management Script

All graph operations use the unified script:

```bash
./scripts/system/manage_graph.sh <command> [options]
```

## Commands

### Start Robot System

Start the robot system with a specific graph:

```bash
# Start with selected graph (from config)
./scripts/system/manage_graph.sh start

# Start with specific graph
./scripts/system/manage_graph.sh start bench_test
./scripts/system/manage_graph.sh start full
```

### Stop Robot System

```bash
./scripts/system/manage_graph.sh stop
```

### Restart Robot System

```bash
# Restart with current graph
./scripts/system/manage_graph.sh restart

# Restart with specific graph
./scripts/system/manage_graph.sh restart bench_test
```

### Select Graph Configuration

Select which graph to use (applies on next start):

```bash
./scripts/system/manage_graph.sh select bench_test
./scripts/system/manage_graph.sh select full
./scripts/system/manage_graph.sh select minimal
```

### Check Status

```bash
./scripts/system/manage_graph.sh status
```

Shows:
- Systemd service status
- Current graph selection
- Running ROS 2 nodes
- Active topics

### View Logs

```bash
./scripts/system/manage_graph.sh logs
```

### Verify Data Streams

Verify all sensor data streams are publishing:

```bash
./scripts/system/manage_graph.sh verify
```

## Available Graphs

- **`minimal`** - System monitor only
- **`full`** - Complete system with all components
- **`robot`** - Default robot configuration
- **`bench_test`** - Bench test configuration (all hardware + visualization)

## Systemd Integration

The graph runtime is managed through systemd:

```bash
# Enable auto-start on boot
sudo systemctl enable isaac-robot.service

# Start service
sudo systemctl start isaac-robot.service

# Check status
sudo systemctl status isaac-robot.service

# View logs
sudo journalctl -u isaac-robot.service -f
```

The service automatically uses the selected graph from `config/robot/selected_graph.txt`.

## Graph Selection Priority

1. **Command argument** - Highest priority (e.g., `manage_graph.sh start bench_test`)
2. **Environment variable** - `ROBOT_GRAPH=bench_test`
3. **Config file** - `config/robot/selected_graph.txt`
4. **Default** - `minimal`

## Examples

### Bench Test Setup

```bash
# Select bench test graph
./scripts/system/manage_graph.sh select bench_test

# Start system
./scripts/system/manage_graph.sh start

# Verify all sensors are streaming
./scripts/system/manage_graph.sh verify

# View logs
./scripts/system/manage_graph.sh logs
```

### Production Deployment

```bash
# Select full graph
./scripts/system/manage_graph.sh select full

# Enable and start service
sudo systemctl enable isaac-robot.service
sudo systemctl start isaac-robot.service

# Check status
./scripts/system/manage_graph.sh status
```

### Development Testing

```bash
# Select minimal graph for testing
./scripts/system/manage_graph.sh select minimal

# Start directly (not via systemd)
./scripts/system/manage_graph.sh start

# Test changes, then restart
./scripts/system/manage_graph.sh restart
```

## Integration with Hardware Setup

Hardware setup and verification is separate:

```bash
# Install hardware components
./scripts/hardware/setup_hardware.sh install all

# Verify hardware is connected
./scripts/hardware/setup_hardware.sh verify

# Then start graph
./scripts/system/manage_graph.sh start bench_test
```

## Troubleshooting

### Service Won't Start

```bash
# Check service status
sudo systemctl status isaac-robot.service

# Check logs
sudo journalctl -u isaac-robot.service -n 50

# Check graph selection
./scripts/system/manage_graph.sh status
```

### No Data Streams

```bash
# Verify hardware is connected
./scripts/hardware/setup_hardware.sh verify

# Verify data streams
./scripts/system/manage_graph.sh verify

# Check ROS 2 topics
ros2 topic list
```

### Graph Not Found

```bash
# List available graphs
ls config/robot/*_graph.yaml

# Select valid graph
./scripts/system/manage_graph.sh select <graph_name>
```

## See Also

- [Hardware Setup](../hardware/HARDWARE_SETUP.md) - Hardware installation
- [Node Management](NODE_MANAGEMENT.md) - Detailed node management
- [Graph Configuration](../robot/GRAPH_CONFIG.md) - Graph configuration details
