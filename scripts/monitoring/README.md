# System Monitoring Scripts

This directory contains scripts for monitoring system health, performance, and hardware status.

## Scripts

### `system_health_check.sh`

Comprehensive system health check.

**Usage**:
```bash
./scripts/monitoring/system_health_check.sh
```

### `monitor_temperatures.sh`

Real-time temperature monitoring with color-coded status.

**Usage**:
```bash
./scripts/monitoring/monitor_temperatures.sh
```

### `monitor_power.sh`

Power consumption monitoring using tegrastats or power sensors.

**Usage**:
```bash
./scripts/monitoring/monitor_power.sh
```

## Installation

All monitoring packages are installed automatically by the unified setup:

```bash
cd ~/src/jetson-orin-nano
./setup.sh
```

See [SYSTEM_MONITOR.md](../../docs/monitoring/SYSTEM_MONITOR.md) for full documentation.
