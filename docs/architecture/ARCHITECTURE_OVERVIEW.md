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
│                    Raw Sensors (High Frequency)              │
│  - IMU (50 Hz)  - Battery (10 Hz)  - Cameras (30 Hz)        │
└───────────────────────┬─────────────────────────────────────┘
                        │
                        ▼
┌─────────────────────────────────────────────────────────────┐
│              nvblox Processor (Full Quality)                 │
│  - Full Resolution Pointclouds                              │
│  - Full Quality Mesh                                         │
│  - Complete TSDF Voxel Grid                                 │
│  - Full Resolution Images                                    │
└───────────────────────┬─────────────────────────────────────┘
                        │
                        ▼
┌─────────────────────────────────────────────────────────────┐
│         Fusion Node (Synchronization + Downsampling)         │
│                                                              │
│  ┌──────────────────────┐  ┌──────────────────────┐        │
│  │  Feature Topics      │  │  Viz Topics          │        │
│  │  (20 MB/s budget)    │  │  (5 MB/s budget)      │        │
│  │                      │  │                      │        │
│  │  - Images: 480×360   │  │  - Image: 320×240    │        │
│  │  - Pointclouds: ×2   │  │  - Pointcloud: ×16   │        │
│  │  - Mesh: 50% decim   │  │  - Mesh: 75% decim   │        │
│  │  - TSDF: 0.1m voxels │  │  - TSDF: Mesh extr   │        │
│  └──────────┬───────────┘  └──────────┬───────────┘        │
└─────────────┼──────────────────────────┼───────────────────┘
              │                          │
              ▼                          ▼
    ┌─────────────────┐        ┌─────────────────┐
    │  Feature Builder│        │  Bridge (Remote)│
    │  (VLA Controller)│        │  (Visualization) │
    └─────────────────┘        └─────────────────┘
```

## Data Flow

### 1. Raw Sensors → nvblox
- High-frequency sensor data flows to nvblox processor
- nvblox computes full-quality 3D representations
- All processing done at maximum quality/resolution

### 2. nvblox → Fusion Node
- Full-quality outputs published to `/nvblox/full/*` topics
- Fusion node subscribes to all full-quality data
- Synchronizes all sensor data to camera frames

### 3. Fusion Node → Feature Topics
- Moderate downsampling for feature builder
- Preserves quality while fitting 20 MB/s budget
- Topics: `/sensor_fusion/*`

### 4. Fusion Node → Viz Topics
- Aggressive downsampling for visualization
- Fits within 5 MB/s bridge budget
- Topics: `/viz/remote/*`

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

1. **nvblox Processor**: Computes full-quality 3D data
2. **Fusion Node**: Synchronizes and downsamples for consumers
3. **Feature Builder**: Consumes feature topics for VLM
4. **Bridge**: Streams visualization topics for remote monitoring

## Benefits

1. **Quality Preservation**: Full quality computation preserves all information
2. **Flexibility**: Adjust downsampling without recomputing
3. **Efficiency**: Compute once, serve many consumers
4. **Bandwidth Control**: Precise control over bandwidth usage
5. **Future-Proof**: Easy to add new consumers with different quality needs
