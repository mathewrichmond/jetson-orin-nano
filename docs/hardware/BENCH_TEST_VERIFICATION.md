# Bench Test Verification Guide

## Overview

This guide walks through verifying all hardware components by launching the ROS graph and streaming data from each sensor using Foxglove Studio.

## Prerequisites

1. **All hardware connected**:
   - Two RealSense cameras (USB)
   - USB microphone
   - ODrive motor controller (serial)
   - iRobot developer kit (serial)

2. **Software installed**:
   - ROS 2 Humble
   - All hardware driver packages built
   - Foxglove Studio installed (on your computer)

3. **Network connectivity**:
   - Robot and your computer on same network
   - Robot IP address known (or mDNS configured)

## Step 1: Verify Hardware

First, run the hardware verification script:

```bash
./scripts/hardware/verify_all_hardware.sh
```

This will check that all hardware is detected and accessible.

## Step 2: Build ROS 2 Packages

Ensure all packages are built:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select isaac_robot realsense_camera usb_microphone odrive_controller irobot_serial
source install/setup.bash
```

## Step 3: Launch Bench Test Configuration

Launch all hardware nodes and visualization bridge using unified management:

```bash
# Select bench test graph
./scripts/system/manage_graph.sh select bench_test

# Start system
./scripts/system/manage_graph.sh start
```

Or start directly with graph:

```bash
./scripts/system/manage_graph.sh start bench_test
```

This will start:
- System monitor
- RealSense cameras (front and rear)
- USB microphone
- ODrive controller
- iRobot serial
- Foxglove bridge (port 8765)

## Step 4: Connect Foxglove Studio

### On Your Computer

1. **Open Foxglove Studio**:
   - Download from [foxglove.dev/download](https://foxglove.dev/download) if needed

2. **Connect to Robot**:
   - Click "Open connection"
   - Select **"ROS 2"** framework
   - Enter robot IP address (or `isaac.local` if mDNS configured)
   - Port: `8765`
   - Click "Open"

3. **Load Layout**:
   - File → Import Layout
   - Select: `config/visualization/foxglove_bench_test_layout.json`
   - Or manually configure panels

## Step 5: Verify Data Streams

### Quick Verification

Run the unified verification command:

```bash
./scripts/system/manage_graph.sh verify
```

This will check that all expected topics are publishing data.

### Manual Verification

Check topics manually:

```bash
# List all topics
ros2 topic list

# Check specific topics
ros2 topic echo /system/status
ros2 topic echo /realsense/status
ros2 topic echo /microphone/status
ros2 topic echo /odrive/status
ros2 topic echo /irobot/status

# Check topic frequency
ros2 topic hz /camera_front/color/image_raw
ros2 topic hz /odrive/imu
ros2 topic hz /irobot/battery
```

## Expected Data Streams

### System Monitor
- `/system/status` - System status messages
- `/system/temperature/cpu` - CPU temperature
- `/system/temperature/gpu` - GPU temperature
- `/system/cpu/usage` - CPU usage percentage
- `/system/memory/usage` - Memory usage percentage

### RealSense Cameras
- `/camera_front/color/image_raw` - Front camera color images
- `/camera_front/depth/image_rect_raw` - Front camera depth images
- `/camera_rear/color/image_raw` - Rear camera color images
- `/camera_rear/depth/image_rect_raw` - Rear camera depth images
- `/realsense/status` - Camera status

### USB Microphone
- `/microphone/status` - Microphone status messages

### ODrive Controller
- `/odrive/status` - ODrive status messages
- `/odrive/imu` - IMU/accelerometer data (linear_acceleration.x/y/z)

### iRobot Serial
- `/irobot/status` - iRobot status messages
- `/irobot/battery` - Battery state (percentage, voltage)

## Visualization Layout

The bench test layout includes:

1. **Camera Views** (4 panels):
   - Front camera color and depth
   - Rear camera color and depth

2. **System Monitoring** (2 plots):
   - CPU/GPU temperature
   - CPU/Memory usage

3. **Sensor Data** (2 plots):
   - ODrive accelerometer (X, Y, Z)
   - iRobot battery (percentage, voltage)

4. **Status Messages** (4 panels):
   - Microphone status
   - ODrive status
   - iRobot status
   - System status

## Troubleshooting

### No Data in Foxglove Studio

1. **Check connection**:
   ```bash
   # On robot, check Foxglove bridge is running
   ros2 node list | grep foxglove

   # Check port is listening
   netstat -tuln | grep 8765
   ```

2. **Check firewall**:
   ```bash
   # Allow port 8765
   sudo ufw allow 8765/tcp
   ```

3. **Verify topics are publishing**:
   ```bash
   ros2 topic list
   ros2 topic echo /system/status
   ```

### Missing Topics

1. **Check nodes are running**:
   ```bash
   ros2 node list
   ```

2. **Check node logs**:
   - Look for errors in terminal where nodes were launched
   - Check for permission errors (serial ports, USB devices)

3. **Verify hardware connections**:
   ```bash
   ./scripts/hardware/verify_all_hardware.sh
   ```

### Camera Not Showing Images

1. **Check camera topics**:
   ```bash
   ros2 topic echo /camera_front/color/image_raw --once
   ```

2. **Check camera node status**:
   ```bash
   ros2 topic echo /realsense/status
   ```

3. **Verify USB connection**:
   ```bash
   lsusb | grep Intel
   ```

### Serial Devices Not Working

1. **Check device permissions**:
   ```bash
   ls -l /dev/ttyUSB*
   groups | grep dialout
   ```

2. **Check device exists**:
   ```bash
   ls -l /dev/ttyUSB* /dev/ttyACM*
   ```

3. **Test serial connection**:
   ```bash
   screen /dev/ttyUSB0 115200
   ```

## Next Steps

Once all data streams are verified:

1. **Test motor commands**:
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
   ```

2. **Record data**:
   ```bash
   ros2 bag record -a
   ```

3. **Integrate with VLA controller** for autonomous operation

## Quick Reference

```bash
# Launch everything
./scripts/hardware/launch_bench_test.sh

# Verify hardware
./scripts/hardware/verify_all_hardware.sh

# Verify data streams
./scripts/hardware/verify_data_streams.sh

# Check topics
ros2 topic list

# Monitor specific topic
ros2 topic echo /system/status
```
