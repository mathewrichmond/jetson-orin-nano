# Isaac Robot System Architecture

## Overview

The Isaac robot system is built on a Jetson Orin Nano running Ubuntu 22.04 and ROS 2 Humble. The system integrates multiple hardware components and uses a Vision-Language-Action (VLA) model as the primary control mechanism.

## System Components

### Core System
- **Hardware**: Jetson Orin Nano Developer Kit
- **OS**: Ubuntu 22.04 (JetPack 5.x)
- **Middleware**: ROS 2 Humble
- **Storage**: microSD (SSD migration planned)

### Final Hardware Setup (Electronics Complete)
- **Cameras**: Two Intel RealSense cameras (USB ports) with hardware frame sync
- **Audio**: USB microphone (USB port)
- **Chassis**: iRobot Create (USB port)
- **Servo + IMU**: SparkFun Auto pHAT via 40-pin GPIO (four servos + ICM-20948 IMU)

### Software Stack
- **System Nodes**: One node per hardware or system device
- **Sensor Fusion**: Time-align and resample all sensor streams
- **Control Planner**: Consumes fused sense vector and outputs time-stamped control plans (not implemented yet)
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
- Control planner (system interface for all control execution)
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

## System Sub-Graph (Stable Interfaces)

The system sub-graph is the stable interface boundary for training and runtime. It is responsible for:

1. **Sense Interface**: Gather all sensor and system data, time-align and resample it, then publish a synchronized fused sense vector to downstream consumers (planner, logging, visualization).
2. **Control Interface**: Consume time-stamped control plans from control sources (onboard model, remote model, or human operator) and execute them as close to real-time as possible.

**Key constraint**: The sense and control interfaces must be stable once model training begins. The internal implementation of the system sub-graph can evolve, but interface changes require model retraining.

### Sense Vector (System Output)
The fused sense vector includes all information needed by downstream consumers:
- Sensor streams (cameras, IMU, chassis)
- System state and health (temperatures, diagnostics, status)
- Synchronization metadata (timestamps, alignment info)

**Excludes**: The control plan itself, which is produced by the control planner.

### Control Plan (System Input)
The control plan is a time-stamped plan that may include:
- Motion commands
- Audio commands
- Mode switches
- Frame rate or frequency changes
- Other system actuation parameters

The control planner is the interface point for all control execution, even if it delegates to other system nodes.

## Data Flow (High-Level)

```
Hardware Nodes → Sensor Fusion → Fused Sense Vector → Consumers (planner, logging, viz)
                                             ↑
                                  Control Plan (time-stamped)
                                             ↑
                     Control Planner (executes control in real time)
```

## Control Modes

The system supports multiple control modes, coordinated through the control planner:

1. **Safe Mode**: All motors disabled, sensors active
2. **Manual Mode**: Direct user control via joystick/interface
3. **Autonomous Mode**: Onboard model active
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
