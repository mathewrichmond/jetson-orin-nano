# Isaac Robot System Architecture

## Overview

The Isaac robot system is built on a Jetson Orin Nano running Ubuntu 22.04 and ROS 2 Humble. The system integrates multiple hardware components and uses a Vision-Language-Action (VLA) model as the primary control mechanism.

## System Components

### Core System
- **Hardware**: Jetson Orin Nano Developer Kit
- **OS**: Ubuntu 22.04 (JetPack 5.x)
- **Middleware**: ROS 2 Humble
- **Storage**: microSD (SSD migration planned)

### Hardware Integration
- **Cameras**: Intel Realsense (D435/D455 planned)
- **Motor Controllers**: TBD
- **Sub-modules**: Raspberry Pi controllers (planned)
- **Sensors**: Additional sensors as needed

### Software Stack
- **VLA Controller**: Custom Vision-Language-Action model
- **Hardware Drivers**: ROS 2 wrappers for hardware interfaces
- **Control Modes**: Multiple operational modes (manual, autonomous, safe, etc.)
- **Monitoring**: System and hardware health monitoring
- **Logging**: Centralized logging infrastructure

## Architecture Layers

### 1. Hardware Layer
- Physical hardware (cameras, motors, sensors)
- Low-level drivers and firmware
- Hardware abstraction interfaces

### 2. Driver Layer
- ROS 2 hardware driver packages
- Hardware-specific configuration
- Device initialization and management

### 3. Control Layer
- VLA model inference
- Control mode switching
- Safety interlocks and checks

### 4. Application Layer
- High-level behaviors and tasks
- Mission planning and execution
- User interfaces

### 5. System Layer
- Monitoring and health checks
- Logging and diagnostics
- Maintenance and recovery

## Data Flow

```
Hardware → Drivers → ROS 2 Topics → Control Modes → VLA Controller → Motor Commands → Hardware
                ↓
         Monitoring & Logging
```

## Control Modes

The system supports multiple control modes:

1. **Safe Mode**: All motors disabled, sensors active
2. **Manual Mode**: Direct user control via joystick/interface
3. **Autonomous Mode**: VLA controller active
4. **Calibration Mode**: Hardware calibration and testing
5. **Recovery Mode**: System recovery and diagnostics

Mode switching includes safety checks and graceful transitions.

## Communication

- **Internal**: ROS 2 topics and services
- **External**: SSH, mDNS (isaac.local)
- **Future**: NFS for shared logging directory

## Safety Considerations

- Emergency stop mechanisms
- Safe mode defaults
- Hardware interlocks
- Watchdog timers
- Resource monitoring

## Future Enhancements

- Distributed control across multiple compute nodes
- Cloud connectivity for model updates
- Advanced monitoring dashboard
- Automated testing framework

