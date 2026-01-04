# API Documentation

## Overview

This document provides API documentation for the Isaac robot system components.

## ROS 2 Topics

### System Topics

- `/system/status` - System health status
- `/system/mode` - Current control mode
- `/system/commands` - System commands

### Hardware Topics

#### Cameras
- `/camera/color/image_raw` - Color images (sensor_msgs/Image)
- `/camera/depth/image_rect_raw` - Depth images (sensor_msgs/Image)
- `/camera/color/camera_info` - Camera calibration (sensor_msgs/CameraInfo)

#### Motors
- `/motors/cmd_vel` - Velocity commands (geometry_msgs/Twist)
- `/motors/status` - Motor status (custom message)
- `/motors/feedback` - Encoder feedback (custom message)

### Control Topics

- `/control/commands` - Control commands
- `/control/state` - Control state
- `/control/mode_switch` - Mode switching requests

## ROS 2 Services

- `/system/set_mode` - Change control mode
- `/system/emergency_stop` - Emergency stop
- `/hardware/calibrate` - Hardware calibration
- `/monitoring/get_status` - Get system status

## ROS 2 Actions

- `/control/execute_task` - Execute a control task
- `/system/update` - System update action

## Configuration Files

Configuration files use YAML format and are located in `config/`:
- System configs: `config/system/`
- Hardware configs: `config/hardware/`
- Control configs: `config/control/`

## Python API

### VLA Controller

```python
from src.vla_controller import VLAController

controller = VLAController(config_path="config/control/vla_config.yaml")
result = controller.infer(image, command)
```

### Hardware Drivers

```python
from src.hardware_drivers import RealsenseDriver, MotorDriver

camera = RealsenseDriver(config="config/hardware/realsense.yaml")
motors = MotorDriver(config="config/hardware/motors.yaml")
```

### Control Modes

```python
from src.control_modes import ControlModeManager

manager = ControlModeManager()
manager.switch_mode("autonomous")
```

## C++ API

*To be documented as C++ components are added*

## Message Types

Custom message types are defined in ROS 2 packages:
- `isaac_msgs/` - Custom message definitions
- See individual packages for message documentation

## Examples

See `docs/examples/` for code examples and tutorials.

