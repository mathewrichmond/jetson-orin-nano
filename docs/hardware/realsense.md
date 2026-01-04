# Intel Realsense Camera Setup

## Overview

This guide covers the installation and configuration of Intel Realsense cameras (D435/D455 series) on the Isaac robot system. The system supports multiple cameras simultaneously with automatic detection and health monitoring.

## Prerequisites

- Jetson Orin Nano with Ubuntu 22.04
- USB 3.0 ports available (2 ports for dual camera setup)
- Adequate power supply for USB devices

## Installation

### Step 1: Install Realsense SDK

```bash
cd ~/src/jetson-orin-nano
sudo ./scripts/hardware/install_realsense.sh
```

This script will:
- Install system dependencies
- Build and install librealsense from source (Jetson-compatible)
- Install pyrealsense2 Python package
- Set up udev rules for USB access
- Optionally clone the official ROS 2 wrapper (if ROS 2 is installed)

### Step 2: Add User to dialout Group

```bash
sudo usermod -a -G dialout $USER
# Log out and back in for changes to take effect
```

### Step 3: Verify Installation

```bash
# Run diagnostics
./scripts/hardware/diagnose_realsense.sh

# Or test manually
python3 -c "import pyrealsense2; print('SDK version:', pyrealsense2.__version__)"
rs-enumerate-devices  # If librealsense tools are installed
```

### Step 4: Build ROS 2 Package

```bash
cd ~/ros2_ws
colcon build --packages-select realsense_camera
source install/setup.bash
```

## Usage

### Launch Cameras

```bash
# Launch with default configuration (auto-detects cameras)
ros2 launch realsense_camera realsense_camera.launch.py

# Or launch as part of full system
ros2 launch isaac_robot full.launch.py
```

### View Camera Data

```bash
# List camera topics
ros2 topic list | grep camera

# View color images
ros2 topic echo /camera_front/color/image_raw
ros2 topic echo /camera_rear/color/image_raw

# View depth images
ros2 topic echo /camera_front/depth/image_rect_raw
ros2 topic echo /camera_rear/depth/image_rect_raw

# Check camera status
ros2 topic echo /realsense/status
```

### Monitor Camera Health

```bash
# Run hardware health check
./monitoring/hardware/realsense_monitor.sh

# Check system monitor camera health topic
ros2 topic echo /system/camera/health
```

## Configuration

Camera parameters are configured in:
- `src/hardware_drivers/realsense_camera/config/realsense_params.yaml`

Key parameters:
- `camera_serial_numbers`: List of specific serial numbers (empty = auto-detect)
- `camera_names`: Names/namespaces for cameras (default: ['camera_front', 'camera_rear'])
- `enable_color`: Enable color stream (default: true)
- `enable_depth`: Enable depth stream (default: true)
- `color_width`, `color_height`, `color_fps`: Color stream parameters
- `depth_width`, `depth_height`, `depth_fps`: Depth stream parameters
- `publish_rate`: Maximum publishing rate in Hz
- `align_depth_to_color`: Align depth to color frame (default: true)

## ROS 2 Topics

For each camera (e.g., `camera_front`, `camera_rear`):

**Color Stream:**
- `/{camera_name}/color/image_raw` - Color images (sensor_msgs/Image)
- `/{camera_name}/color/camera_info` - Camera calibration (sensor_msgs/CameraInfo)

**Depth Stream:**
- `/{camera_name}/depth/image_rect_raw` - Depth images (sensor_msgs/Image)
- `/{camera_name}/depth/camera_info` - Depth camera calibration (sensor_msgs/CameraInfo)

**Optional:**
- `/{camera_name}/points` - Pointcloud (sensor_msgs/PointCloud2, if enabled)

**Status:**
- `/realsense/status` - Camera status messages (std_msgs/String)
- `/system/camera/health` - Camera health monitoring (std_msgs/String)

## Integration

The RealSense cameras are integrated into:

1. **ROS Graph**: Defined in `config/robot/robot_graph.yaml`
2. **Launch Files**: Included in `src/isaac_robot/launch/full.launch.py`
3. **Health Monitoring**: Monitored by `system_monitor` node
4. **Hardware Monitoring**: Script at `monitoring/hardware/realsense_monitor.sh`

## Troubleshooting

- **Camera not detected**:
  - Check USB connection: `lsusb | grep Intel`
  - Verify USB 3.0 connection (check `dmesg` for USB speed)
  - Try different USB ports

- **Permission errors**:
  - Add user to dialout group: `sudo usermod -a -G dialout $USER`
  - Log out and back in
  - Check udev rules: `ls -l /etc/udev/rules.d/99-realsense-libusb.rules`

- **Low FPS or dropped frames**:
  - Ensure USB 3.0 connection (check with `lsusb -t`)
  - Reduce resolution/framerate in config
  - Check USB bandwidth: `dmesg | grep -i usb`

- **Power issues**:
  - Ensure adequate power supply
  - Use powered USB hub if needed
  - Check power consumption: `tegrastats`

- **ROS topics not publishing**:
  - Check camera status: `ros2 topic echo /realsense/status`
  - Verify cameras are detected: `./scripts/hardware/diagnose_realsense.sh`
  - Check node is running: `ros2 node list`

## Diagnostics

Run comprehensive diagnostics:

```bash
./scripts/hardware/diagnose_realsense.sh
```

This checks:
- SDK installation
- USB device detection
- USB permissions
- Camera connectivity
- ROS 2 wrapper availability

## Next Steps

- Configure camera parameters for your specific use case
- Integrate camera data with VLA controller
- Adjust resolution/framerate based on performance requirements
- Set up camera-specific namespaces if needed
