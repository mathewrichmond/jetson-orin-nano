# System Monitor ROS 2 Package

ROS 2 package for comprehensive system monitoring of the Isaac robot.

## Features

- Temperature monitoring (CPU, GPU, all thermal zones)
- Power consumption tracking
- CPU, memory, and disk usage monitoring
- Threshold-based alerting
- ROS 2 topic publishing

## Quick Start

1. Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select system_monitor
source install/setup.bash
```

2. Launch the monitor:
```bash
ros2 launch system_monitor system_monitor.launch.py
```

3. Monitor topics:
```bash
ros2 topic echo /system/status
ros2 topic echo /system/temperature/cpu
ros2 topic echo /system/alerts
```

## Dependencies

- `psutil` - System and process utilities
- ROS 2 Humble
- Python 3

Install psutil if needed:
```bash
pip3 install psutil
```

## Configuration

Edit `config/monitor_config.yaml` to adjust thresholds.

See `docs/monitoring/SYSTEM_MONITOR.md` for full documentation.

