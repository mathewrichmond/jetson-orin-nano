# Bench Setup Quick Reference

## Overview

This document provides a quick reference for verifying and using all hardware components in the Isaac robot bench setup.

## Hardware Components

1. **Two RealSense Cameras (USB)** - Vision sensors
2. **USB Microphone** - Audio input
3. **ODrive Motor Controller and Accelerometer** - Motor control and IMU
4. **iRobot Developer Kit Serial Connection (USB)** - Mobile base

## Quick Verification

Run the comprehensive hardware verification script:

```bash
./scripts/hardware/verify_all_hardware.sh
```

This will check all four hardware components and provide a detailed report.

## Individual Component Checks

### RealSense Cameras

```bash
# Check USB devices
lsusb | grep -i "8086:0b07\|8086:0b5c\|8086:0b64"

# Run diagnostics
./scripts/hardware/diagnose_realsense.sh

# Test ROS 2 node
ros2 launch realsense_camera realsense_camera.launch.py
ros2 topic echo /realsense/status
```

### USB Microphone

```bash
# Check USB devices
lsusb | grep -i "audio\|microphone\|mic"

# Check PulseAudio
pactl list sources short

# Test recording
arecord -d 5 test.wav
aplay test.wav

# Test ROS 2 node
ros2 launch usb_microphone usb_microphone.launch.py
ros2 topic echo /microphone/status
```

### ODrive Motor Controller

```bash
# Check serial devices
ls -l /dev/ttyUSB* /dev/ttyACM*

# Check USB devices (if USB-connected)
lsusb | grep -i "odrive\|1209:0d32"

# Test ROS 2 node
ros2 launch odrive_controller odrive_controller.launch.py
ros2 topic echo /odrive/status
ros2 topic echo /odrive/imu
```

### iRobot Developer Kit

```bash
# Check serial devices
ls -l /dev/ttyUSB* /dev/ttyACM*

# Check USB devices
lsusb | grep -i "irobot\|create\|roomba"

# Test ROS 2 node
ros2 launch irobot_serial irobot_serial.launch.py
ros2 topic echo /irobot/status
ros2 topic echo /irobot/battery
```

## Launch All Hardware

Launch all hardware nodes using the bench test graph:

```bash
# Launch bench test configuration
ros2 launch isaac_robot robot.launch.py \
    graph_config:=bench_test_graph.yaml \
    group:=bench_test
```

Or launch individual nodes:

```bash
# Terminal 1: System monitor
ros2 run system_monitor system_monitor_node

# Terminal 2: RealSense cameras
ros2 launch realsense_camera realsense_camera.launch.py

# Terminal 3: USB microphone
ros2 launch usb_microphone usb_microphone.launch.py

# Terminal 4: ODrive controller
ros2 launch odrive_controller odrive_controller.launch.py

# Terminal 5: iRobot serial
ros2 launch irobot_serial irobot_serial.launch.py
```

## Monitor All Topics

```bash
# List all topics
ros2 topic list

# Monitor status topics
ros2 topic echo /system/status
ros2 topic echo /realsense/status
ros2 topic echo /microphone/status
ros2 topic echo /odrive/status
ros2 topic echo /irobot/status

# Monitor sensor data
ros2 topic echo /odrive/imu
ros2 topic echo /irobot/battery
ros2 topic echo /camera_front/color/image_raw
```

## Common Issues

### Serial Port Conflicts

If multiple serial devices are connected, they may appear as `/dev/ttyUSB0`, `/dev/ttyUSB1`, etc. Check which device is which:

```bash
# Before connecting device
ls -l /dev/ttyUSB*

# After connecting device
ls -l /dev/ttyUSB*

# Compare to identify new device
```

Update configuration files accordingly:
- ODrive: `config/hardware/odrive_params.yaml` - `serial_port`
- iRobot: `config/hardware/irobot_params.yaml` - `serial_port`

### USB Power Issues

If devices are not detected or disconnect frequently:

1. **Check USB power**:
   ```bash
   dmesg | grep -i usb
   ```

2. **Use powered USB hub** if needed

3. **Check USB bandwidth**:
   ```bash
   lsusb -t
   ```

### Permission Errors

Ensure user is in required groups:

```bash
# Check groups
groups

# Add to dialout (for serial devices)
sudo usermod -a -G dialout $USER

# Add to audio (for microphone)
sudo usermod -a -G audio $USER

# Log out and back in for changes to take effect
```

## Configuration Files

All hardware configuration files are in `config/hardware/`:

- `realsense_params.yaml` - RealSense camera configuration
- `microphone_params.yaml` - USB microphone configuration
- `odrive_params.yaml` - ODrive motor controller configuration
- `irobot_params.yaml` - iRobot serial configuration

## Graph Configuration

Bench test graph configuration: `config/robot/bench_test_graph.yaml`

This includes:
- System monitor
- RealSense cameras
- USB microphone
- ODrive controller
- iRobot serial
- Foxglove bridge (for visualization)

## Next Steps

1. **Verify all hardware** using the verification script
2. **Test each component** individually
3. **Launch full bench test** configuration
4. **Monitor topics** to verify data flow
5. **Integrate with VLA controller** for autonomous operation

## Documentation

- [Hardware Setup Guide](HARDWARE_SETUP.md) - Detailed setup instructions
- [RealSense Setup](realsense.md) - Camera configuration
- [ODrive Setup](odrive.md) - Motor controller configuration
- [USB Microphone Setup](usb_microphone.md) - Microphone configuration
- [iRobot Setup](irobot.md) - Robot base configuration
