# Unified Management System

## Overview

The Isaac robot system uses a **unified management approach** that eliminates ad-hoc scripts and provides consistent, systemd-integrated runtime management.

## Key Principles

1. **Single Entry Points** - All operations go through unified scripts
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

```bash
./scripts/system/manage_graph.sh <command> [options]
```

**Commands:**
- `start [graph]` - Start robot system with graph
- `stop` - Stop robot system
- `restart [graph]` - Restart robot system
- `status` - Show system status
- `select [graph]` - Select graph configuration
- `logs` - Show system logs
- `verify` - Verify data streams

**Examples:**
```bash
# Start bench test
./scripts/system/manage_graph.sh start bench_test

# Check status
./scripts/system/manage_graph.sh status

# Verify data streams
./scripts/system/manage_graph.sh verify
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
sudo systemctl enable isaac-robot.service

# Start service
sudo systemctl start isaac-robot.service

# Check status
sudo systemctl status isaac-robot.service

# View logs
sudo journalctl -u isaac-robot.service -f
```

The service automatically uses the selected graph from `config/robot/selected_graph.txt`.

## Graph Selection

Select graphs using the unified script:

```bash
./scripts/system/manage_graph.sh select bench_test
```

Or use the utility script:

```bash
./scripts/utils/select_graph.sh bench_test
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
./scripts/system/manage_graph.sh select bench_test

# 2. Start system
./scripts/system/manage_graph.sh start

# 3. Verify data streams
./scripts/system/manage_graph.sh verify
```

### Production Deployment

```bash
# 1. Select production graph
./scripts/system/manage_graph.sh select full

# 2. Enable and start service
sudo systemctl enable isaac-robot.service
sudo systemctl start isaac-robot.service

# 3. Monitor
./scripts/system/manage_graph.sh status
```

## Removed Ad-Hoc Scripts

The following ad-hoc scripts have been removed and integrated:

- ❌ `scripts/hardware/launch_bench_test.sh` → Use `manage_graph.sh start bench_test`
- ❌ `scripts/hardware/launch_bench_test_direct.sh` → Use `manage_graph.sh start bench_test`
- ❌ `scripts/hardware/verify_data_streams.sh` → Use `manage_graph.sh verify`

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
