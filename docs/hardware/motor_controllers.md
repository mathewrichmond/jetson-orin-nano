# Motor Controller Setup

## Overview

This guide covers the integration of motor controllers with the Isaac robot system.

## Supported Controllers

*To be determined based on hardware selection*

## General Setup Process

1. **Hardware Connection**: Connect motor controllers via CAN, I2C, or serial
2. **Driver Installation**: Install controller-specific drivers
3. **ROS 2 Interface**: Create ROS 2 driver package
4. **Configuration**: Configure controller parameters
5. **Testing**: Verify motor control functionality

## Communication Interfaces

### CAN Bus
- Tools: `can-utils` (installed via setup script)
- Check CAN interface: `ip link show can0`
- Test: `candump can0`

### I2C
- Tools: `i2c-tools` (installed via setup script)
- Scan bus: `i2cdetect -y 1`
- Read registers: `i2cdump -y 1 <address>`

### Serial/UART
- Check devices: `ls /dev/tty*`
- Permissions: Add user to `dialout` group

## ROS 2 Integration

Motor controllers should be integrated via ROS 2 topics:
- `/motors/cmd_vel` - Velocity commands
- `/motors/status` - Motor status feedback
- `/motors/feedback` - Encoder/position feedback

## Safety Considerations

- Always implement emergency stop functionality
- Use safe mode as default
- Implement watchdog timers
- Test in safe environment first

## Configuration Files

Motor controller configurations are stored in:
- `config/hardware/motor_controllers/` - Controller-specific configs
- `hardware/motor_controllers/` - Setup scripts and documentation

## Troubleshooting

- **No communication**: Check wiring, power, interface configuration
- **Permission errors**: Check user groups and permissions
- **Control not working**: Verify ROS 2 topics and message types
- **Safety issues**: Ensure emergency stop is functional

## Next Steps

- Select motor controller hardware
- Create driver package in `src/hardware_drivers/`
- Implement safety checks and interlocks

