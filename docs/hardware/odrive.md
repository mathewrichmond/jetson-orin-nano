# ODrive Motor Controller Setup

## Overview

This guide covers the integration of ODrive motor controllers with the Isaac robot system. ODrive controllers provide high-performance motor control with built-in accelerometer/IMU support.

## Prerequisites

- Jetson Orin Nano with Ubuntu 22.04
- ODrive motor controller (v3.6 or compatible)
- Serial connection (USB) or CAN bus interface
- Adequate power supply for motors

## Hardware Connection

### Serial Connection (USB)

1. **Connect ODrive via USB**:
   - Connect ODrive to Jetson via USB cable
   - ODrive typically appears as `/dev/ttyUSB0` or `/dev/ttyACM0`

2. **Check device**:
   ```bash
   ls -l /dev/ttyUSB* /dev/ttyACM*
   ```

3. **Verify permissions**:
   ```bash
   groups | grep dialout
   ```
   If not in dialout group:
   ```bash
   sudo usermod -a -G dialout $USER
   # Log out and back in
   ```

### CAN Bus Connection (Alternative)

1. **Connect CAN interface**:
   - Connect ODrive to CAN bus
   - Configure CAN interface: `sudo ip link set can0 up type can bitrate 500000`

2. **Test CAN connection**:
   ```bash
   candump can0
   ```

## Software Installation

### Install Dependencies

```bash
sudo apt update
sudo apt install -y python3-pip python3-serial
pip3 install pyserial
```

### Build ROS 2 Package

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select odrive_controller
source install/setup.bash
```

## Configuration

### Serial Configuration

Edit `config/hardware/odrive_params.yaml`:

```yaml
/**:
  ros__parameters:
    serial_port: '/dev/ttyUSB0'  # Adjust based on your device
    baudrate: 115200
    timeout: 1.0
    use_can: false
    publish_rate: 50.0
    enable_accelerometer: true
```

### CAN Configuration

If using CAN:

```yaml
/**:
  ros__parameters:
    use_can: true
    can_interface: 'can0'
    can_node_id: 0
    enable_accelerometer: true
```

## Usage

### Launch ODrive Node

```bash
# Launch standalone
ros2 launch odrive_controller odrive_controller.launch.py

# Or launch as part of full system
ros2 launch isaac_robot robot.launch.py graph_config:=robot_graph.yaml group:=hardware
```

### Monitor Status

```bash
# Check ODrive status
ros2 topic echo /odrive/status

# Check IMU data
ros2 topic echo /odrive/imu

# List all topics
ros2 topic list | grep odrive
```

### Send Commands

```bash
# Publish velocity command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## ROS 2 Topics

### Published Topics

- `/odrive/status` (std_msgs/String) - ODrive status messages
- `/odrive/imu` (sensor_msgs/Imu) - Accelerometer/IMU data (if enabled)

### Subscribed Topics

- `/cmd_vel` (geometry_msgs/Twist) - Velocity commands

## Troubleshooting

### ODrive Not Detected

1. **Check USB connection**:
   ```bash
   lsusb | grep -i odrive
   ```

2. **Check serial device**:
   ```bash
   ls -l /dev/ttyUSB* /dev/ttyACM*
   ```

3. **Check permissions**:
   ```bash
   groups | grep dialout
   ```

### Communication Errors

1. **Verify baudrate**: ODrive typically uses 115200 baud
2. **Check serial port**: Ensure correct `/dev/ttyUSB*` device
3. **Test serial connection**: Use `minicom` or `screen` to test:
   ```bash
   screen /dev/ttyUSB0 115200
   ```

### CAN Connection Issues

1. **Check CAN interface**:
   ```bash
   ip link show can0
   ```

2. **Verify CAN is up**:
   ```bash
   sudo ip link set can0 up type can bitrate 500000
   ```

3. **Test CAN communication**:
   ```bash
   candump can0
   ```

## Safety Considerations

- **Emergency Stop**: Always implement emergency stop functionality
- **Safe Mode**: Use safe mode as default
- **Watchdog Timers**: Implement watchdog timers for safety
- **Test First**: Test in safe environment before full operation

## Next Steps

- Configure motor parameters in ODrive firmware
- Calibrate motors and encoders
- Test motor control with low speeds first
- Integrate with VLA controller for autonomous operation
