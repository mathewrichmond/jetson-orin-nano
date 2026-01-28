# Hardware Integration Guide

This document provides an overview of hardware components integrated with the Isaac robot system.

## Final Hardware Setup (Electronics Complete)

The Isaac robot system uses the following hardware configuration:

### Main Computer
- **Jetson Orin Nano**: Primary compute platform
  - Ubuntu 22.04 (JetPack 5.x)
  - ROS 2 Humble framework

### Cameras
- **Two Intel RealSense Cameras**: D435/D455 series (USB ports)
  - Hardware frame synchronization via ground and sync pin connections
  - Setup: Via unified setup or `scripts/utils/hardware_manager.py`
  - Drivers: Intel RealSense SDK 2.0 + ROS 2 wrapper
  - Documentation: [RealSense Setup](realsense.md) | [Hardware Setup Guide](HARDWARE_SETUP.md) | [Camera Frame Sync](CAMERA_FRAME_SYNC.md)

### Motor Control & IMU
- **SparkFun Auto pHAT**: Connected via 40-pin GPIO header
  - Setup: See [SparkFun Auto pHAT Setup](sparkfun_auto_phat.md)
  - Communication: I2C (GPIO header)
  - Features:
    - Four servo motor outputs (for camera actuation)
    - ICM-20948 9DOF IMU (accelerometer, gyroscope, magnetometer)
  - Documentation: [SparkFun Auto pHAT Setup](sparkfun_auto_phat.md)

### Audio Input
- **USB Microphone**: USB audio input device
  - Setup: See [USB Microphone Setup](usb_microphone.md)
  - Communication: USB audio (ALSA/PulseAudio)
  - Features: Audio capture for voice commands
  - Documentation: [USB Microphone Setup](usb_microphone.md)

### Robot Base
- **iRobot Create**: Mobile base chassis
  - Setup: See [iRobot Setup](irobot.md)
  - Communication: Serial (USB port)
  - Features: Mobile base control, battery monitoring
  - Documentation: [iRobot Setup](irobot.md)

## Hardware Setup Process

Hardware components can be installed through the unified setup or individually:

1. **Physical Installation**: Mount and connect hardware
2. **Driver Installation**:
   - Via unified setup: `./setup.sh` (answer 'yes' to hardware prompts)
   - Via hardware manager: `sudo python3 scripts/utils/hardware_manager.py install-<component>`
3. **Configuration**: Configure hardware parameters
4. **Testing**: Verify hardware operation
5. **Integration**: Integrate with ROS 2 system

See [Hardware Setup Guide](HARDWARE_SETUP.md) for detailed instructions.

## Hardware-Specific Documentation

### Hardware Organization

- **[BOM.md](BOM.md)** - Bill of Materials (complete parts list)
- **[ELECTRICAL.md](ELECTRICAL.md)** - Electrical specifications, wiring, power requirements
- **[MECHANICAL.md](MECHANICAL.md)** - Mechanical specifications, tolerances, calibration values

### Setup & Installation

- [Hardware Setup Guide](HARDWARE_SETUP.md) - Unified hardware installation guide
- [RealSense Setup](realsense.md) - Camera installation and configuration
- [Camera Frame Sync](CAMERA_FRAME_SYNC.md) - Hardware frame synchronization setup
- [SparkFun Auto pHAT Setup](sparkfun_auto_phat.md) - Servo control and IMU setup
- [USB Microphone Setup](usb_microphone.md) - USB microphone setup
- [iRobot Setup](irobot.md) - iRobot Create setup

## Power Considerations

- Jetson Orin Nano power budget: ~15W
- USB devices (Realsense): Require adequate power supply
- Motor controllers: May require separate power supply
- Monitor power consumption: `scripts/monitoring/system/power_monitor.sh`

## USB Device Management

- Check USB devices: `lsusb`
- USB power: Ensure adequate power supply for USB devices
- USB bandwidth: Realsense cameras require USB 3.0

## I2C/SPI/CAN Interfaces

- I2C tools: `i2cdetect`, `i2cdump` (installed via setup script)
- CAN tools: `candump`, `cansend` (installed via setup script)
- Check available interfaces: `ip link show`

## Troubleshooting

- **Device not detected**: Check connections, power, USB ports
- **Permission errors**: Add user to appropriate groups (dialout, i2c, etc.)
- **Driver issues**: Check kernel modules and driver installation
- **Performance issues**: Check USB bandwidth, power supply, thermal throttling

## Hardware Connection Summary

- **Jetson Orin Nano** (main computer)
  - **USB Ports**: Two RealSense cameras, USB microphone, iRobot Create
  - **40-pin GPIO**: SparkFun Auto pHAT (servos and IMU)
  - **Camera Sync**: Ground and sync pins connected between cameras for hardware frame synchronization
