# System Bringup - Quick Reference

**Last Verified**: 2026-01-27  
**Status**: ✅ Operational

---

## Quick Start

### 1. Build (if needed)
```bash
cd /home/nano/src/jetson-orin-nano
colcon build --symlink-install
source install/setup.bash
```

### 2. Launch Minimal System
```bash
# System monitoring only (always works)
ros2 launch isaac_robot graph.launch.py \
  graph_config:=minimal_graph.yaml \
  group:=minimal
```

### 3. Check Health
```bash
# In another terminal
ros2 topic echo /system/health/summary --once
ros2 topic echo /system/cpu/usage --once
ros2 topic echo /system/temperature/cpu --once
```

---

## Modular Graph Testing

### Core Modules
```bash
# System monitoring (no hardware required)
ros2 launch isaac_robot graph.launch.py \
  graph_config:=modular_graph.yaml \
  group:=core

# Chassis control (requires IMU hardware or mock mode)
ros2 launch isaac_robot graph.launch.py \
  graph_config:=modular_graph.yaml \
  group:=chassis_control

# Power management (requires GPIO or mock mode)
ros2 launch isaac_robot graph.launch.py \
  graph_config:=modular_graph.yaml \
  group:=power_management
```

### Vision & Sensing
```bash
# Vision pipeline (requires RealSense camera)
ros2 launch isaac_robot graph.launch.py \
  graph_config:=modular_graph.yaml \
  group:=vision_pipeline

# Audio pipeline (requires USB microphone)
ros2 launch isaac_robot graph.launch.py \
  graph_config:=modular_graph.yaml \
  group:=audio_pipeline
```

### Planning & Control
```bash
# VLA planner (requires vision + audio)
ros2 launch isaac_robot graph.launch.py \
  graph_config:=modular_graph.yaml \
  group:=vla_planner
```

---

## Available Groups

| Group | Nodes | Hardware Required |
|-------|-------|-------------------|
| `core` | system_monitor, health_monitor | None ✅ |
| `chassis_control` | imu_processor, chassis_controller, calibration_manager | IMU (can use mock) |
| `power_management` | power_manager, gpio_controller, battery_monitor | GPIO (can use mock) |
| `vision_pipeline` | visual_slam, camera_calibration, vision_pipeline | RealSense camera |
| `audio_pipeline` | audio_feature_extractor, speech_recognition, audio_pipeline | USB microphone |
| `vla_planner` | vla_controller, action_executor, planner | Vision + Audio |
| `hardware` | realsense_camera, nvblox_processor, irobot_serial, phat_motor | All hardware |

---

## Monitoring Commands

### Node Status
```bash
# List running nodes
ros2 node list

# Get node info
ros2 node info /system/system_monitor
```

### Topic Monitoring
```bash
# List all topics
ros2 topic list

# Echo a topic (once)
ros2 topic echo /system/health/summary --once

# Monitor topic rate
ros2 topic hz /system/cpu/usage

# View topic type
ros2 topic info /system/health/summary
```

### Health Checks
```bash
# System performance
ros2 topic echo /system/cpu/usage --once
ros2 topic echo /system/memory/usage --once
ros2 topic echo /system/temperature/cpu --once
ros2 topic echo /system/temperature/gpu --once

# Node health
ros2 topic echo /system/health/nodes --once
ros2 topic echo /system/health/summary --once

# Chassis (if running)
ros2 topic echo /rpi/imu/filtered --once
ros2 topic echo /rpi/chassis/odometry --once
ros2 topic echo /rpi/calibration/status --once
```

---

## Troubleshooting

### Build Fails
```bash
# Clean build
rm -rf build/ install/ log/
colcon build --symlink-install
```

### NumPy Version Issues
```bash
# Check versions
python3 -c "import numpy, cv2; print(f'NumPy: {numpy.__version__}, OpenCV: {cv2.__version__}')"

# Fix if needed (downgrade to 1.x)
pip3 install "numpy<2" --user
```

### Launch Parameter Issues
**Wrong**: `graph:=minimal_graph.yaml` ❌  
**Correct**: `graph_config:=minimal_graph.yaml` ✅

### Hardware Not Available
Use `mock_mode:=true` for nodes that support it:
```bash
ros2 launch power_management power_management.launch.py mock_mode:=true
```

---

## System Logs

### ROS Logs
```bash
# Latest log directory
ls -lt ~/.ros/log/ | head -5

# View logs
less ~/.ros/log/latest/system_monitor_node-1-stdout.log
```

### Systemd Service
```bash
# Status
systemctl --user status isaac-robot.service

# Logs
journalctl --user -u isaac-robot.service -f

# Restart
systemctl --user restart isaac-robot.service
```

---

## Next Steps

### Bench Testing (No Hardware)
1. ✅ Test core system (system_monitor, health_monitor)
2. ⏸️ Test chassis_control with mock IMU data
3. ⏸️ Test power_management with mock GPIO
4. ⏸️ Test individual nodes with `ros2 run`

### Hardware Integration
1. ⏸️ Connect RealSense camera → Test vision_pipeline
2. ⏸️ Connect USB microphone → Test audio_pipeline  
3. ⏸️ Connect iRobot serial → Test irobot_serial
4. ⏸️ Connect motor controller → Test phat_motor_controller
5. ⏸️ Run full integration test

### System Integration
1. ⏸️ Update systemd service configuration
2. ⏸️ Test distributed launch (dual-compute)
3. ⏸️ Test remote deployment
4. ⏸️ Run integration test suite

---

## Quick Commands Reference

```bash
# Build
colcon build --symlink-install && source install/setup.bash

# Launch minimal
ros2 launch isaac_robot graph.launch.py graph_config:=minimal_graph.yaml group:=minimal

# Check nodes
ros2 node list

# Check health
ros2 topic echo /system/health/summary --once

# Stop all
pkill -f "ros2 launch"
```

---

## See Also

- **Full Report**: `SYSTEM_VERIFICATION_REPORT.md`
- **Testing Guide**: `docs/testing/TESTING.md`
- **Deployment**: `docs/deployment/DEPLOYMENT.md`
- **Architecture**: `docs/architecture/IMPLEMENTATION_STATUS.md`

---

**Status**: System operational ✅  
**Last Updated**: 2026-01-27
