# RealSense Camera ROS 2 Package

ROS 2 package for Intel RealSense depth cameras (D435/D455 series).

## Features

- Supports multiple cameras simultaneously
- Publishes color and depth images
- Publishes camera calibration info
- Optional pointcloud generation
- Configurable resolution and framerate
- Health monitoring integration

## Topics

For each camera (e.g., `camera_front`, `camera_rear`):

- `/{camera_name}/color/image_raw` - Color images (sensor_msgs/Image)
- `/{camera_name}/color/camera_info` - Color camera calibration (sensor_msgs/CameraInfo)
- `/{camera_name}/depth/image_rect_raw` - Depth images (sensor_msgs/Image)
- `/{camera_name}/depth/camera_info` - Depth camera calibration (sensor_msgs/CameraInfo)
- `/{camera_name}/points` - Pointcloud (sensor_msgs/PointCloud2, optional)

Status:
- `/realsense/status` - Camera status messages (std_msgs/String)

## Parameters

- `camera_serial_numbers` - List of camera serial numbers (empty = auto-detect all)
- `camera_names` - List of camera names/namespaces (default: ['camera_front', 'camera_rear'])
- `enable_color` - Enable color stream (default: true)
- `enable_depth` - Enable depth stream (default: true)
- `enable_pointcloud` - Enable pointcloud publishing (default: false)
- `color_width` - Color image width (default: 640)
- `color_height` - Color image height (default: 480)
- `color_fps` - Color framerate (default: 30)
- `depth_width` - Depth image width (default: 640)
- `depth_height` - Depth image height (default: 480)
- `depth_fps` - Depth framerate (default: 30)
- `publish_rate` - Maximum publishing rate in Hz (default: 30.0)
- `align_depth_to_color` - Align depth to color frame (default: true)

## Usage

### Launch single node (auto-detects cameras)

```bash
ros2 launch realsense_camera realsense_camera.launch.py
```

### Launch with custom config

```bash
ros2 launch realsense_camera realsense_camera.launch.py config_file:=/path/to/config.yaml
```

### View topics

```bash
ros2 topic list | grep camera
ros2 topic echo /camera_front/color/image_raw
ros2 topic echo /camera_front/depth/image_rect_raw
```

## Building

```bash
cd ~/ros2_ws
colcon build --packages-select realsense_camera
source install/setup.bash
```

## Dependencies

- pyrealsense2 (Python RealSense SDK)
- cv_bridge (OpenCV bridge for ROS)
- sensor_msgs
- image_transport

## Troubleshooting

- **No cameras detected**: Check USB connections, run `lsusb` to verify cameras are connected
- **Permission errors**: Add user to dialout group: `sudo usermod -a -G dialout $USER` (then log out/in)
- **Low FPS**: Check USB 3.0 connection, reduce resolution/framerate
- **Camera not publishing**: Check `/realsense/status` topic for error messages
