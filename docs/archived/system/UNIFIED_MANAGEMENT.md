# Unified Management System

## Overview

The Isaac robot system uses a **unified management approach** that eliminates ad-hoc scripts and provides consistent, systemd-integrated runtime management.

## Key Principles

1. **Single Entry Point** - Systemd user service for production
2. **Systemd Integration** - Runtime managed through systemd services
3. **State Tracking** - Setup state is tracked and idempotent
4. **Consistent Interface** - Same commands work across all environments

## Unified Scripts

### Hardware Management

```bash
./scripts/hardware/setup_hardware.sh <command> [options]
```

**Commands:**
- `install [component]` - Install hardware components
- `verify` - Verify all hardware is connected
- `diagnose [component]` - Run component diagnostics
- `status` - Show hardware status
- `list` - List available components

**Examples:**
```bash
# Install all hardware
./scripts/hardware/setup_hardware.sh install all

# Verify hardware
./scripts/hardware/setup_hardware.sh verify

# Check status
./scripts/hardware/setup_hardware.sh status
```

### Graph Management

**Production (systemd user service):**
```bash
systemctl --user start isaac-robot.service
systemctl --user status isaac-robot.service
journalctl --user -u isaac-robot.service -f
```

**Development (direct launch):**
```bash
systemctl --user stop isaac-robot.service
ros2 launch isaac_robot composable_graph.launch.py \
  graph_config:=robot_graph.yaml \
  group:=sensor_pipeline \
  use_composable:=true
```

## Unified Setup

All setup goes through the main setup script:

```bash
./setup.sh
```

This handles:
- System packages
- Python packages
- ROS 2 workspace
- Hardware installation
- Hardware verification
- Service installation

Setup is **idempotent** - safe to run multiple times.

## Systemd Integration

Runtime is managed through systemd:

```bash
# Enable auto-start
systemctl --user enable isaac-robot.service

# Start service
systemctl --user start isaac-robot.service

# Check status
systemctl --user status isaac-robot.service

# View logs
journalctl --user -u isaac-robot.service -f
```

The service automatically uses the selected graph from `config/robot/selected_graph.txt`.

## Graph Selection

Select graphs by setting the selection file:

```bash
echo "bench_test" > config/robot/selected_graph.txt
```

Available graphs:
- `minimal` - System monitor only
- `full` - Complete system
- `robot` - Default robot configuration
- `bench_test` - Bench test (all hardware + visualization)

## Workflow

### Initial Setup

```bash
# 1. Run unified setup
./setup.sh

# 2. Install hardware (if needed)
./scripts/hardware/setup_hardware.sh install all

# 3. Verify hardware
./scripts/hardware/setup_hardware.sh verify
```

### Running Bench Test

```bash
# 1. Select bench test graph
echo "bench_test" > config/robot/selected_graph.txt

# 2. Start system
systemctl --user start isaac-robot.service

# 3. Verify data streams
ros2 topic list
```

### Production Deployment

```bash
# 1. Select production graph
echo "full" > config/robot/selected_graph.txt

# 2. Enable and start service
systemctl --user enable isaac-robot.service
systemctl --user start isaac-robot.service

# 3. Monitor
systemctl --user status isaac-robot.service
```

## Removed Ad-Hoc Scripts

The following ad-hoc scripts have been removed and integrated:

- ❌ `scripts/hardware/launch_bench_test.sh` → Use `systemctl --user start isaac-robot.service`
- ❌ `scripts/hardware/launch_bench_test_direct.sh` → Use `systemctl --user start isaac-robot.service`
- ❌ `scripts/hardware/verify_data_streams.sh` → Use `ros2 topic list`

All functionality is now available through unified scripts.

## Benefits

1. **Consistency** - Same interface for all operations
2. **Maintainability** - Single source of truth for each operation
3. **Systemd Integration** - Proper service management
4. **State Tracking** - Setup state is tracked and idempotent
5. **Documentation** - Clear, unified documentation

## See Also

- [Graph Management](GRAPH_MANAGEMENT.md) - Detailed graph management guide
- [Hardware Setup](../hardware/HARDWARE_SETUP.md) - Hardware installation guide
- [Node Management](NODE_MANAGEMENT.md) - Node management details
