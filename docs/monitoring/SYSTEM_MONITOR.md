# System Monitor Documentation

## Overview

The System Monitor module provides comprehensive monitoring of the Isaac robot system, focusing on critical metrics for robot operation: temperature, power consumption, CPU usage, memory, and disk space.

## Features

- **Temperature Monitoring**: CPU, GPU, and all thermal zones
- **Power Monitoring**: Real-time power consumption tracking
- **Resource Monitoring**: CPU, memory, and disk usage
- **Alerting**: Threshold-based alerts for critical conditions
- **ROS 2 Integration**: Publishes status via ROS 2 topics
- **Per-node Health**: Nodes publish structured health heartbeats
- **Health Aggregation**: Health monitor aggregates node status into system health

## Installation

### 1. Run Unified Setup

The unified setup script installs all required packages:

```bash
cd ~/src/jetson-orin-nano
./setup.sh
```

This installs system packages, Python packages, and sets up the ROS 2 workspace automatically.

### 2. Build ROS 2 Package

```bash
cd ~/ros2_ws
# Link or copy system_monitor package
ln -s ~/src/jetson-orin-nano/src/system_monitor src/
# Or copy it:
# cp -r ~/src/jetson-orin-nano/src/system_monitor src/

rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select system_monitor
source install/setup.bash
```

## Usage

### ROS 2 Node

Launch the system monitor node:

```bash
ros2 launch system_monitor system_monitor.launch.py
```

With custom parameters:

```bash
ros2 launch system_monitor system_monitor.launch.py \
    update_rate:=2.0 \
    temp_warning:=75.0 \
    temp_critical:=90.0
```

### Standalone Scripts

#### Temperature Monitoring

```bash
./scripts/monitoring/monitor_temperatures.sh
```

Continuously monitors all thermal zones with color-coded status.

#### Power Monitoring

```bash
./scripts/monitoring/monitor_power.sh
```

Monitors power consumption using tegrastats or power sensors.

#### System Health Check

```bash
./scripts/monitoring/system_health_check.sh
```

One-time comprehensive health check.

## ROS 2 Topics

The system monitor publishes the following topics:

- `/system/status` (`std_msgs/String`) - Human-readable status summary
- `/system/temperature/cpu` (`sensor_msgs/Temperature`) - CPU temperature
- `/system/temperature/gpu` (`sensor_msgs/Temperature`) - GPU temperature
- `/system/cpu/usage` (`std_msgs/Float32`) - CPU usage percentage
- `/system/memory/usage` (`std_msgs/Float32`) - Memory usage percentage
- `/system/disk/usage` (`std_msgs/Float32`) - Disk usage percentage
- `/system/power` (`std_msgs/Float32`) - Power consumption in watts
- `/system/alerts` (`std_msgs/String`) - Alert messages for threshold violations
- `/system/health/summary` (`std_msgs/String`) - Aggregated health summary (JSON)
- `/system/health/nodes` (`std_msgs/String`) - Per-node health payloads (JSON)

Each node publishes a health heartbeat on its namespace:

- `/<namespace>/health/<node_name>` (`std_msgs/String`) - Per-node health (JSON)

## Configuration

Edit `src/system_monitor/config/monitor_config.yaml` to adjust thresholds:

```yaml
system_monitor:
  ros__parameters:
    update_rate: 1.0  # Hz
    temp_warning_threshold: 70.0  # °C
    temp_critical_threshold: 85.0  # °C
    cpu_warning_threshold: 80.0  # %
    memory_warning_threshold: 85.0  # %
    disk_warning_threshold: 85.0  # %
```

## Monitoring Thresholds

### Temperature

- **Warning**: 70°C (default) - System is getting warm
- **Critical**: 85°C (default) - Risk of thermal throttling

### Resource Usage

- **CPU Warning**: 80% (default)
- **Memory Warning**: 85% (default)
- **Disk Warning**: 85% (default)

## Thermal Zones

The Jetson Orin Nano has multiple thermal zones:
- `cpu-thermal` - CPU temperature
- `gpu-thermal` - GPU temperature
- `cv0-thermal`, `cv1-thermal`, `cv2-thermal` - CV (Computer Vision) cores
- `soc0-thermal`, `soc1-thermal`, `soc2-thermal` - SOC components
- `tj-thermal` - Junction temperature

## Power Monitoring

Power monitoring uses:
1. INA3221 power sensors (if accessible)
2. tegrastats (Jetson-specific tool)
3. Falls back if neither is available

To install jetson_stats for enhanced power monitoring:
```bash
sudo pip3 install -U jetson-stats
```

## Troubleshooting

### Temperature readings unavailable

- Check thermal zone permissions: `ls -l /sys/class/thermal/thermal_zone*/temp`
- Some zones may require root access
- Some zones may be temporarily unavailable

### Power readings unavailable

- Install jetson_stats: `sudo pip3 install -U jetson-stats`
- Check power sensor paths: `ls /sys/bus/i2c/drivers/ina3221x/`
- May require root access for some sensors

### ROS 2 topics not publishing

- Verify node is running: `ros2 node list`
- Check for errors: `ros2 topic echo /system/status`
- Verify package is built: `colcon build --packages-select system_monitor`

## Integration with Sensor Fusion

The system monitor is integrated into the **system sub-graph** and its status is included in the **fused sense vector** (`/sensor_fusion/system/*`). This ensures:

- **Single Point of Understanding**: Same system status information is available to both the controller and visualization
- **Stable Interface**: System status is part of the stable sense vector interface
- **Bandwidth Control**: Frequency can be adjusted for different consumers (controller vs visualization)
- **Comprehensive Status**: Includes traditional metrics (CPU, GPU, memory) AND system-specific status (camera sync, device errors, voltages)

The sensor fusion node consumes system monitor topics and includes them in the fused output, synchronized with sensor data.

Health summaries are also fused for downstream consumers:

- `/sensor_fusion/system/health/summary`
- `/sensor_fusion/system/health/nodes`
- `/viz/remote/system/health/summary`
- `/viz/remote/system/health/nodes`

These provide low-rate health visibility for remote monitoring.

## Health Monitoring Design

### Per-node Health

Each node publishes a JSON health payload that includes:
- Current status (`OK`, `WARN`, `ERROR`, `INIT`)
- Watchdog freshness (last seen age, expected rates, timeouts)
- Recent error/warning messages

Nodes maintain watchdogs for:
- Internal publish loops (heartbeat)
- Critical input topics (staleness + rate checks)

### Health Monitor Node

The `health_monitor_node` aggregates all per-node health topics into:
- `/system/health/summary` (overall status + stale/error nodes)
- `/system/health/nodes` (raw per-node payloads)

The health monitor reads the active graph configuration and respects the
rates and tolerances defined in the running graph parameters.

## Integration

The system monitor can be integrated with:
- **Sensor Fusion Node**: System status included in fused sense vector
- **Control Planner**: Consumes system status as part of sense vector
- **Visualization**: System status available for monitoring dashboards
- **Logging**: System status recorded alongside sensor data
- **Alert Systems**: Threshold-based alerts for notifications

## Future Enhancements

- [ ] Network I/O rate monitoring (rx/tx rates)
- [ ] USB device monitoring and status
- [ ] Camera sync status integration
- [ ] Device error state tracking
- [ ] Missing hardware detection
- [ ] Voltage monitoring (if sensors available)
- [ ] Historical data logging
- [ ] Web-based dashboard
- [ ] Email/SMS alerts
- [ ] Predictive thermal management
- [ ] Power mode recommendations
