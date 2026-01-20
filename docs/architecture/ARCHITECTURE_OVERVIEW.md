# System Architecture Overview

This document provides a high-level overview of the Isaac robot system architecture.

## Core Philosophy

**Compute Once, Downsample for Consumers**

The system computes everything at full quality, then downsamples/decimates to fit bandwidth budgets for different consumers. This approach:
- Preserves quality where bandwidth allows
- Provides flexibility to adjust quality without recomputing
- Enables multiple consumers with different quality needs
- Maximizes computational efficiency

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Raw Sensors & System State                │
│  - IMU  - Chassis  - Cameras  - System Monitor              │
│  - Hardware Status  - Device Errors  - Sync Status         │
└───────────────────────┬─────────────────────────────────────┘
                        │
                        ▼
┌─────────────────────────────────────────────────────────────┐
│        Sensor Fusion Node (Sync + Resample + Align)          │
│  - Time-align all streams                                   │
│  - Resample to uniform rates                                │
│  - Fuse sensor data + system status                        │
│  - Publish unified sense vector                             │
└───────────────────────┬─────────────────────────────────────┘
                        │
                        ▼
┌─────────────────────────────────────────────────────────────┐
│     Fused Sense Vector (Stable Interface)                    │
│  - Sensor streams (cameras, IMU, chassis)                  │
│  - System status (CPU, GPU, memory, temps, errors)         │
│  - Hardware status (sync, devices, voltages)               │
│  - Same data for controller & visualization                │
│  - Frequency controlled for bandwidth                      │
└───────────────────────┬─────────────────────────────────────┘
                        │
                        ▼
┌─────────────────────────────────────────────────────────────┐
│       Control Planner (Stable Interface, TBD)                │
│  - Consumes fused sense vector                              │
│  - Emits time-stamped control plans                         │
└───────────────────────┬─────────────────────────────────────┘
                        │
                        ▼
┌─────────────────────────────────────────────────────────────┐
│          Control Execution (Real-Time)                       │
│  - Motion, audio, mode switches, rate changes               │
└─────────────────────────────────────────────────────────────┘
```

## Data Flow

### 1. Raw Sensors & System State → Sensor Fusion
- Hardware nodes publish raw sensor streams (cameras, IMU, chassis)
- System monitor publishes system status (CPU, GPU, memory, disk, network I/O)
- Hardware-specific status (camera sync, device errors, temperatures, voltages)
- Fusion node time-aligns, resamples, and synchronizes all inputs

### 2. Sensor Fusion → Fused Sense Vector
- Unified, stable interface for all downstream consumers
- Includes sensor streams AND comprehensive system status
- **Single Point of Understanding**: Same system status exposed to controller and visualization
- Frequency controlled for bandwidth (higher for controller, lower for visualization)

### 3. Fused Sense Vector → Control Planner
- Planner consumes the sense vector
- Planner outputs time-stamped control plans

### 4. Control Planner → Control Execution
- System executes control plans as close to real time as possible

## Bandwidth Management

### Feature Builder Budget: 20 MB/s
- **Target**: ~18 MB/s (leave margin)
- **Current**: ~20.2 MB/s
- **Components**: Images, pointclouds, mesh, TSDF, IMU, chassis

### Bridge Budget: 5 MB/s
- **Target**: ~4 MB/s (leave margin)
- **Current**: ~3.2 MB/s
- **Components**: Image, pointcloud, mesh, TSDF mesh, IMU, chassis, system

## Graph Configurations

### `robot` Graph
- Full system with visualization
- Bridge enabled
- All downsampling active
- **Use**: Development, testing, remote monitoring

### `minimal` Graph
- Production mode
- Bridge disabled
- Visualization topics disabled
- Feature topics only
- **Use**: Autonomous operation, production deployment

## Key Components

1. **Sensor Fusion Node**: Synchronizes and resamples all data
2. **Control Planner Node**: Stable control interface (not implemented yet)
3. **System Nodes**: Per-device drivers for hardware and system state
4. **Consumers**: Logging, visualization, and planning

## Benefits

1. **Quality Preservation**: Full quality computation preserves all information
2. **Flexibility**: Adjust downsampling without recomputing
3. **Efficiency**: Compute once, serve many consumers
4. **Bandwidth Control**: Precise control over bandwidth usage
5. **Future-Proof**: Easy to add new consumers with different quality needs
