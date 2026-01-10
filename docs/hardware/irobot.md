# iRobot Developer Kit Setup

## Overview

This guide covers the integration of iRobot Create/Roomba robots with the Isaac robot system via serial communication. The iRobot Developer Kit provides a mobile base platform for the robot.

## Prerequisites

- Jetson Orin Nano with Ubuntu 22.04
- iRobot Create 2 or Roomba (with serial interface)
- USB-to-serial adapter (if needed)
- Adequate power supply

## Hardware Connection

### Serial Connection

1. **Connect iRobot via serial**:
   - Connect iRobot Create/Roomba to Jetson via USB-to-serial adapter
   - Device typically appears as `/dev/ttyUSB0` or `/dev/ttyACM0`

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

### iRobot Create 2 Specifics

- **Baudrate**: 115200 (default for Create 2)
- **Protocol**: Open Interface (OI) Protocol
- **Power**: Ensure iRobot is powered on and in good state

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
colcon build --packages-select irobot_serial
source install/setup.bash
```

## Configuration

Edit `config/hardware/irobot_params.yaml`:

```yaml
/**:
  ros__parameters:
    serial_port: '/dev/ttyUSB0'  # Adjust based on your device
    baudrate: 115200
    timeout: 1.0
    robot_type: 'create2'  # Options: create2, roomba
    publish_rate: 10.0  # Hz
```

### Finding Serial Port

To find the correct serial port:

```bash
# List serial devices
ls -l /dev/ttyUSB* /dev/ttyACM*

# Test serial connection
screen /dev/ttyUSB0 115200
```

## Usage

### Launch iRobot Node

```bash
# Launch standalone
ros2 launch irobot_serial irobot_serial.launch.py

# Or launch as part of full system
ros2 launch isaac_robot robot.launch.py graph_config:=robot_graph.yaml group:=hardware
```

### Monitor Status

```bash
# Check iRobot status
ros2 topic echo /irobot/status

# Check battery status
ros2 topic echo /irobot/battery

# List all topics
ros2 topic list | grep irobot
```

### Send Commands

```bash
# Publish velocity command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.1}}"

# Stop robot
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## ROS 2 Topics

### Published Topics

- `/irobot/status` (std_msgs/String) - iRobot status messages
- `/irobot/battery` (sensor_msgs/BatteryState) - Battery state information

### Subscribed Topics

- `/cmd_vel` (geometry_msgs/Twist) - Velocity commands

## iRobot Create 2 Protocol

The node implements the iRobot Create 2 Open Interface (OI) Protocol:

- **Start Command**: 128 - Starts OI
- **Safe Mode**: 131 - Enables safe mode (default)
- **Drive Command**: 137 - Sets velocity and radius
- **Stop Command**: 173 - Stops all movement

### Velocity Commands

- **Linear velocity**: -500 to 500 mm/s
- **Angular velocity**: Controlled via radius (-2000 to 2000 mm)
- **Straight**: Radius = 32767

## Troubleshooting

### iRobot Not Detected

1. **Check USB connection**:
   ```bash
   lsusb | grep -i "irobot\|create\|roomba"
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

1. **Verify baudrate**: iRobot Create 2 uses 115200 baud
2. **Check serial port**: Ensure correct `/dev/ttyUSB*` device
3. **Test serial connection**: Use `screen` or `minicom`:
   ```bash
   screen /dev/ttyUSB0 115200
   ```

### Robot Not Responding

1. **Check power**: Ensure iRobot is powered on
2. **Check battery**: Low battery may prevent operation
3. **Reset connection**: Unplug and replug USB cable
4. **Check OI mode**: Robot must be in correct OI mode

## Safety Considerations

- **Safe Mode**: Node initializes robot in safe mode by default
- **Emergency Stop**: Always implement emergency stop functionality
- **Battery Monitoring**: Monitor battery status to prevent unexpected shutdowns
- **Test First**: Test with low speeds in safe environment

## Battery Information

The node publishes battery state:
- **Percentage**: 0.0 to 1.0 (0% to 100%)
- **Voltage**: Approximate voltage based on charge
- **Max Charge**: Typically ~16000 (iRobot Create 2)

## Next Steps

- Integrate with VLA controller for autonomous navigation
- Add sensor data reading (bumps, wheel drops, etc.)
- Implement safety behaviors (cliff detection, etc.)
- Add odometry publishing for localization
