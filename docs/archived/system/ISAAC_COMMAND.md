# Isaac Command Reference

The `isaac` command provides a centralized interface for managing the robot system, graphs, and services.

## Quick Start

```bash
# Start robot
./scripts/system/isaac start

# Stop robot
./scripts/system/isaac stop

# Switch graph
./scripts/system/isaac switch full

# Check status
./scripts/system/isaac status
```

## Commands

### `isaac start`

Start the robot system with the currently selected graph.

```bash
./scripts/system/isaac start
```

- Uses graph from `config/robot/selected_graph.txt`
- Starts via systemd service if enabled, otherwise launches directly
- Automatically sources ROS 2 and workspace

### `isaac stop`

Stop all running robot nodes.

```bash
./scripts/system/isaac stop
```

- Stops systemd service if running
- Stops any direct launches
- Cleans up ROS 2 processes

### `isaac restart`

Restart the robot system with the currently selected graph.

```bash
./scripts/system/isaac restart
```

- Stops current nodes
- Starts with selected graph
- Useful after configuration changes

### `isaac status`

Show current system status.

```bash
./scripts/system/isaac status
```

Shows:
- Selected graph
- Systemd service status
- Running ROS 2 nodes
- Active topics

### `isaac graph`

Show or select graph.

```bash
# Show current graph
./scripts/system/isaac graph

# Switch graph and restart
./scripts/system/isaac graph full
```

### `isaac switch <graph>`

Switch to a different graph (without restarting).

```bash
./scripts/system/isaac switch minimal
./scripts/system/isaac switch full
./scripts/system/isaac switch robot
```

- Updates `config/robot/selected_graph.txt`
- Updates systemd service configuration
- Does NOT restart (use `isaac graph <name>` to switch and restart)

### `isaac logs`

View node logs.

```bash
./scripts/system/isaac logs
```

- Shows live logs if service is running
- Shows recent logs if service is stopped

### `isaac service <action>`

Manage systemd service.

```bash
./scripts/system/isaac service enable   # Enable auto-start on boot
./scripts/system/isaac service disable  # Disable auto-start
./scripts/system/isaac service start    # Start service
./scripts/system/isaac service stop     # Stop service
./scripts/system/isaac service restart  # Restart service
./scripts/system/isaac service status   # Show service status
```

## Graph Selection

Graphs are stored in `config/robot/selected_graph.txt`:

- **`minimal`** - Essential nodes only (system monitor)
- **`full`** - Complete system with all components
- **`robot`** - Configurable graph with custom settings

### Priority Order

1. Environment variable `ROBOT_GRAPH` (highest priority)
2. Config file `config/robot/selected_graph.txt`
3. Default: `minimal`

## Service Integration

When you switch graphs, the systemd service is automatically updated:

```bash
# Switch graph
./scripts/system/isaac switch full

# Service will use new graph on next start/restart
./scripts/system/isaac service restart
```

The service ensures nodes stay running and automatically restarts them if they crash.

## Examples

### Development Workflow

```bash
# Start with minimal graph
./scripts/system/isaac switch minimal
./scripts/system/isaac start

# Switch to full system
./scripts/system/isaac graph full

# Check status
./scripts/system/isaac status
```

### Production Setup

```bash
# Enable auto-start
./scripts/system/isaac service enable

# Set production graph
./scripts/system/isaac switch full

# Start service
./scripts/system/isaac service start

# Verify
./scripts/system/isaac status
```

### Quick Testing

```bash
# Test different graphs
./scripts/system/isaac graph minimal  # Switch and restart
./scripts/system/isaac graph full     # Switch and restart
./scripts/system/isaac graph robot    # Switch and restart
```

## Integration with Other Tools

The `isaac` command integrates with:

- **Graph selection** - Uses `config/robot/selected_graph.txt`
- **Systemd services** - Manages `isaac-robot.service`
- **Node management** - Coordinates with `manage_nodes.sh`
- **Visualization** - Can be used alongside `start_visualization.sh`

## See Also

- [Node Management](NODE_MANAGEMENT.md)
- [Graph Configuration](../robot/GRAPH_CONFIG.md)
- [Graph Selection](../robot/GRAPH_SELECTION.md)
