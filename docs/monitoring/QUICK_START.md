# System Monitor Quick Start

## Installation

### 1. Run Unified Setup

Run the unified setup script (installs all packages):

```bash
./setup.sh
```

### 2. Build ROS 2 Package

```bash
cd ~/ros2_ws/src
ln -s ~/src/jetson-orin-nano/src/system_monitor .
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select system_monitor
source install/setup.bash
```

## Usage

### Quick Temperature Check

```bash
./scripts/monitoring/monitor_temperatures.sh
```

### Quick Power Check

```bash
./scripts/monitoring/monitor_power.sh
```

### Full System Health Check

```bash
./scripts/monitoring/system_health_check.sh
```

### ROS 2 Monitoring Node

```bash
# Launch the monitor
ros2 launch system_monitor system_monitor.launch.py

# In another terminal, check status
ros2 topic echo /system/status
ros2 topic echo /system/temperature/cpu
ros2 topic echo /system/alerts
```

## Next Steps

- Configure WiFi: `nmtui` or `nmcli device wifi connect <SSID> password <password>`
- Test display: Connect display and verify it works
- Review monitoring documentation: `docs/monitoring/SYSTEM_MONITOR.md`

