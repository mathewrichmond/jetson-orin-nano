# Bench Setup Quick Reference

## Overview

This document provides a quick reference for verifying and using all hardware components in the Isaac robot bench setup.

## Hardware Components

1. **Two RealSense Cameras (USB ports)** - Vision sensors with hardware frame sync
2. **USB Microphone (USB port)** - Audio input
3. **SparkFun Auto pHAT (40-pin GPIO)** - Four servo motors for camera actuation and ICM-20948 IMU
4. **iRobot Create (USB port)** - Mobile base chassis

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

### SparkFun Auto pHAT

```bash
# Check I2C devices
i2cdetect -y 7

# Check ROS 2 node
ros2 launch phat_motor_controller phat_motor_controller.launch.py
ros2 topic echo /phat/status
ros2 topic echo /phat/imu
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

# Terminal 4: SparkFun Auto pHAT
ros2 launch phat_motor_controller phat_motor_controller.launch.py

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
ros2 topic echo /phat/status
ros2 topic echo /irobot/status

# Monitor sensor data
ros2 topic echo /phat/imu
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
- SparkFun Auto pHAT: `config/hardware/phat_params.yaml` - I2C bus and IMU address
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
- `phat_params.yaml` - SparkFun Auto pHAT configuration (servos and IMU)
- `irobot_params.yaml` - iRobot Create serial configuration

## Graph Configuration

Bench test graph configuration: `config/robot/bench_test_graph.yaml`

This includes:
- System monitor
- RealSense cameras (hardware frame sync)
- USB microphone
- SparkFun Auto pHAT (servos and IMU)
- iRobot Create serial
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
- [Camera Frame Sync](CAMERA_FRAME_SYNC.md) - Hardware frame synchronization
- [SparkFun Auto pHAT Setup](sparkfun_auto_phat.md) - Servo and IMU configuration
- [USB Microphone Setup](usb_microphone.md) - Microphone configuration
- [iRobot Setup](irobot.md) - Robot base configuration
