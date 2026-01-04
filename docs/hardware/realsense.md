# Intel Realsense Camera Setup

## Overview

This guide covers the installation and configuration of Intel Realsense cameras (D435/D455 series) on the Isaac robot system.

## Prerequisites

- Jetson Orin Nano with Ubuntu 22.04
- USB 3.0 port available
- Adequate power supply for USB devices

## Installation

### Step 1: Install Realsense SDK

```bash
cd ~/src/jetson-orin-nano
sudo ./scripts/hardware/install_realsense.sh
```

### Step 2: Install ROS 2 Wrapper

```bash
cd ~/ros2_ws/src
git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-development
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select realsense2_camera
```

### Step 3: Configure Camera

Edit configuration files in `config/hardware/realsense/`:
- `realsense_params.yaml` - Camera parameters
- `realsense_launch.py` - Launch file configuration

## Testing

```bash
# Test SDK installation
realsense-viewer

# Test ROS 2 integration
ros2 launch realsense2_camera rs_launch.py
ros2 topic list
ros2 topic echo /camera/color/image_raw
```

## Configuration

Camera parameters can be configured in:
- `config/hardware/realsense/realsense_params.yaml`
- Launch file parameters in `hardware/realsense/launch/`

## Troubleshooting

- **Camera not detected**: Check USB connection, try different USB port
- **Permission errors**: Add user to `dialout` group: `sudo usermod -a -G dialout $USER`
- **Low FPS**: Check USB 3.0 connection, reduce resolution/framerate
- **Power issues**: Ensure adequate power supply, use powered USB hub if needed

## ROS 2 Topics

The Realsense ROS 2 wrapper publishes:
- `/camera/color/image_raw` - Color images
- `/camera/depth/image_rect_raw` - Depth images
- `/camera/color/camera_info` - Camera calibration info
- `/camera/depth/camera_info` - Depth camera info

## Next Steps

- Integrate camera data with VLA controller
- Configure camera parameters for your use case
- Set up multiple cameras if needed

