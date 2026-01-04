# Hardware Integration Guide

This document provides an overview of hardware components integrated with the Isaac robot system.

## Supported Hardware

### Cameras
- **Intel Realsense**: D435/D455 series (planned)
  - Setup: See `hardware/realsense/` directory
  - Drivers: Intel Realsense SDK 2.0 + ROS 2 wrapper

### Motor Controllers
- **TBD**: Motor controller models to be determined
  - Setup: See `hardware/motor_controllers/` directory
  - Communication: CAN, I2C, or serial (TBD)

### Sub-module Controllers
- **Raspberry Pi**: For distributed control (planned)
  - Setup: See `hardware/raspberry_pi_modules/` directory
  - Communication: Network-based (ROS 2 over network)

## Hardware Setup Process

1. **Physical Installation**: Mount and connect hardware
2. **Driver Installation**: Run hardware-specific setup scripts
3. **Configuration**: Configure hardware parameters
4. **Testing**: Verify hardware operation
5. **Integration**: Integrate with ROS 2 system

## Hardware-Specific Documentation

- [Realsense Setup](realsense.md) - Camera installation and configuration
- [Motor Controllers](motor_controllers.md) - Motor controller setup
- [Raspberry Pi Modules](raspberry_pi_modules.md) - Sub-module integration

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

## Future Hardware

- Additional sensors (IMU, LiDAR, etc.)
- Actuators (grippers, manipulators, etc.)
- Communication modules (WiFi, Bluetooth, etc.)

