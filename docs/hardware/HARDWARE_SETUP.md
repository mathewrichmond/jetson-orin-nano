# Hardware Setup Guide

## Overview

This guide covers hardware component installation and configuration for the Isaac robot system. Hardware components are managed through the unified setup system and can be installed individually or as part of the full setup.

## Quick Start

### Install All Hardware Components

Run the unified setup and answer 'yes' to hardware component prompts:

```bash
./setup.sh
```

### Install Individual Components

Use the hardware manager CLI:

```bash
# Install RealSense cameras
sudo python3 scripts/utils/hardware_manager.py install-realsense

# Build RealSense ROS package
python3 scripts/utils/hardware_manager.py build-realsense-ros

# Run diagnostics
python3 scripts/utils/hardware_manager.py diagnose-realsense

# List available components
python3 scripts/utils/hardware_manager.py list
```

## RealSense Cameras

### Installation

RealSense cameras are installed through the unified setup or manually:

**Via Unified Setup:**
```bash
./setup.sh
# Answer 'yes' when prompted for RealSense installation
```

**Manual Installation:**
```bash
sudo python3 scripts/utils/hardware_manager.py install-realsense
```

This installs:
- Intel RealSense SDK 2.0 (librealsense)
- Python bindings (pyrealsense2)
- ROS 2 wrapper (realsense-ros)
- udev rules for USB access

### Post-Installation

After installation, add your user to the dialout group for USB access:

```bash
sudo usermod -a -G dialout $USER
# Log out and back in for changes to take effect
```

### Building ROS Package

The RealSense ROS package is automatically built during setup, but you can rebuild manually:

```bash
python3 scripts/utils/hardware_manager.py build-realsense-ros
```

Or manually:
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select realsense_camera
source install/setup.bash
```

### Verification

Run diagnostics to verify cameras are working:

```bash
python3 scripts/utils/hardware_manager.py diagnose-realsense
```

Or use the shell script:
```bash
./scripts/hardware/diagnose_realsense.sh
```

### Usage

Launch cameras with ROS 2:

```bash
# Launch cameras only
ros2 launch realsense_camera realsense_camera.launch.py

# Launch as part of full system
ros2 launch isaac_robot full.launch.py
```

View camera topics:
```bash
ros2 topic list | grep camera
ros2 topic echo /camera_front/color/image_raw
ros2 topic echo /realsense/status
```

## Hardware Manager CLI

The hardware manager (`scripts/utils/hardware_manager.py`) provides a unified interface for managing hardware components:

### Commands

- `install-realsense` - Install RealSense SDK and ROS wrapper
- `build-realsense-ros` - Build RealSense ROS 2 package
- `diagnose-realsense` - Run camera diagnostics
- `list` - List available hardware components

### State Management

The hardware manager tracks installation state in `.hardware_state`:

```bash
# View installation state
cat .hardware_state

# Reset state (to reinstall)
rm .hardware_state
```

## Troubleshooting

### RealSense Installation Fails

1. Check dependencies:
   ```bash
   sudo apt-get update
   sudo apt-get install -y build-essential cmake git libssl-dev libusb-1.0-0-dev
   ```

2. Check USB permissions:
   ```bash
   groups  # Should include 'dialout'
   ```

3. Verify cameras are detected:
   ```bash
   lsusb | grep Intel
   ```

### ROS Package Build Fails

1. Ensure ROS 2 workspace exists:
   ```bash
   ls ~/ros2_ws/src
   ```

2. Check package is linked:
   ```bash
   ls -la ~/ros2_ws/src/realsense_camera
   ```

3. Install dependencies:
   ```bash
   cd ~/ros2_ws
   source /opt/ros/humble/setup.bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

### Cameras Not Detected

1. Check USB connection:
   ```bash
   lsusb | grep 8086:0b07
   ```

2. Check USB speed (should be 3.0):
   ```bash
   lsusb -t
   ```

3. Check udev rules:
   ```bash
   ls -l /etc/udev/rules.d/99-realsense-libusb.rules
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ```

## ODrive Motor Controller

### Installation

ODrive controllers are integrated via ROS 2:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select odrive_controller
source install/setup.bash
```

### Configuration

Configure ODrive in `config/hardware/odrive_params.yaml`:
- Set serial port (e.g., `/dev/ttyUSB0`)
- Configure baudrate (typically 115200)
- Enable accelerometer if available

### Verification

```bash
# Launch ODrive node
ros2 launch odrive_controller odrive_controller.launch.py

# Check status
ros2 topic echo /odrive/status
```

See [ODrive Setup](odrive.md) for detailed configuration.

## USB Microphone

### Installation

USB microphones are integrated via ROS 2:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select usb_microphone
source install/setup.bash
```

### Configuration

Configure microphone in `config/hardware/microphone_params.yaml`:
- Set device name (use `arecord -l` to find device)
- Configure sample rate and format
- Set chunk size and publish rate

### Verification

```bash
# Launch microphone node
ros2 launch usb_microphone usb_microphone.launch.py

# Check status
ros2 topic echo /microphone/status

# Test recording
arecord -d 5 test.wav
```

See [USB Microphone Setup](usb_microphone.md) for detailed configuration.

## iRobot Developer Kit

### Installation

iRobot Create/Roomba integration via ROS 2:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select irobot_serial
source install/setup.bash
```

### Configuration

Configure iRobot in `config/hardware/irobot_params.yaml`:
- Set serial port (e.g., `/dev/ttyUSB1`)
- Configure baudrate (115200 for Create 2)
- Set robot type (create2 or roomba)

### Verification

```bash
# Launch iRobot node
ros2 launch irobot_serial irobot_serial.launch.py

# Check status
ros2 topic echo /irobot/status
ros2 topic echo /irobot/battery
```

See [iRobot Setup](irobot.md) for detailed configuration.

## Hardware Verification

Run the comprehensive hardware verification script:

```bash
./scripts/hardware/verify_all_hardware.sh
```

This script checks:
- Two RealSense cameras (USB)
- USB microphone
- ODrive motor controller and accelerometer
- iRobot developer kit serial connection

## Next Steps

- See [RealSense Setup](realsense.md) for detailed camera configuration
- See [ODrive Setup](odrive.md) for motor controller configuration
- See [USB Microphone Setup](usb_microphone.md) for microphone configuration
- See [iRobot Setup](irobot.md) for robot base configuration
- See [Hardware Integration](../hardware/HARDWARE.md) for hardware overview
- See [System Setup](../setup/SETUP.md) for full system setup
