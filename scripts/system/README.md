# System Management Scripts

Centralized scripts for managing the Isaac robot system.

## Main Command: `isaac`

The `isaac` command provides a unified interface for all robot management:

```bash
# Start robot with selected graph
./scripts/system/isaac start

# Stop robot
./scripts/system/isaac stop

# Restart robot
./scripts/system/isaac restart

# Show status
./scripts/system/isaac status

# Show current graph
./scripts/system/isaac graph

# Switch graph and restart
./scripts/system/isaac graph full

# Switch graph (without restart)
./scripts/system/isaac switch robot

# Manage systemd service
./scripts/system/isaac service enable   # Enable auto-start
./scripts/system/isaac service start    # Start service
./scripts/system/isaac service status  # Check status

# View logs
./scripts/system/isaac logs
```

## Graph Management

### Available Graphs

- **`minimal`** - Essential nodes only (system monitor)
- **`full`** - Complete system with all components
- **`robot`** - Configurable graph with custom settings

### Selecting a Graph

```bash
# Show current graph
./scripts/system/isaac graph

# Switch graph (updates config, doesn't restart)
./scripts/system/isaac switch full

# Switch graph and restart
./scripts/system/isaac graph robot
```

The selected graph is stored in `config/robot/selected_graph.txt` and persists across reboots.

## Service Management

The robot can run as a systemd service for automatic startup:

```bash
# Enable service (starts on boot)
./scripts/system/isaac service enable

# Start service now
./scripts/system/isaac service start

# Stop service
./scripts/system/isaac service stop

# Restart service
./scripts/system/isaac service restart

# Check status
./scripts/system/isaac service status

# Disable service
./scripts/system/isaac service disable
```

## Other Scripts

- `start_robot.sh` - Low-level robot startup (used by service)
- `start_visualization.sh` - Launch visualization tools
- `manage_nodes.sh` - Advanced node management (legacy, use `isaac` instead)

## Examples

### Development Workflow

```bash
# Start with minimal graph
./scripts/system/isaac switch minimal
./scripts/system/isaac start

# Switch to full system for testing
./scripts/system/isaac graph full

# Check what's running
./scripts/system/isaac status
```

### Production Setup

```bash
# Enable auto-start on boot
./scripts/system/isaac service enable

# Set production graph
./scripts/system/isaac switch full

# Start service
./scripts/system/isaac service start

# Verify
./scripts/system/isaac status
```

### Quick Graph Switching

```bash
# Switch and restart in one command
./scripts/system/isaac graph minimal  # Switches to minimal and restarts
./scripts/system/isaac graph full     # Switches to full and restarts
```
