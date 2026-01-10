# Bench Test Quick Start

## Unified Setup and Management

All operations use the unified management system:

### 1. Setup Hardware

```bash
# Install all hardware components
./scripts/hardware/setup_hardware.sh install all

# Verify hardware is connected
./scripts/hardware/setup_hardware.sh verify
```

### 2. Launch Bench Test

```bash
# Select bench test graph
./scripts/system/manage_graph.sh select bench_test

# Start system
./scripts/system/manage_graph.sh start
```

### 3. Verify Data Streams

```bash
./scripts/system/manage_graph.sh verify
```

### 4. Connect Foxglove Studio

1. Open Foxglove Studio
2. Connect to robot IP (or `isaac.local`) on port 8765
3. Import layout: `config/visualization/foxglove_bench_test_layout.json`

## Systemd Management

For production use:

```bash
# Enable auto-start
sudo systemctl enable isaac-robot.service

# Start service
sudo systemctl start isaac-robot.service

# Check status
./scripts/system/manage_graph.sh status

# View logs
./scripts/system/manage_graph.sh logs
```

## Quick Reference

```bash
# Hardware management
./scripts/hardware/setup_hardware.sh install all
./scripts/hardware/setup_hardware.sh verify
./scripts/hardware/setup_hardware.sh status

# Graph management
./scripts/system/manage_graph.sh select bench_test
./scripts/system/manage_graph.sh start
./scripts/system/manage_graph.sh verify
./scripts/system/manage_graph.sh status
```

See [Bench Test Verification Guide](docs/hardware/BENCH_TEST_VERIFICATION.md) for details.
