# Documentation

## Core Docs

- **[ARCHITECTURE.md](ARCHITECTURE.md)** - Articulated cameras, UDM worldgraph, digital twin
- **[CALIBRATION.md](CALIBRATION.md)** - Fixture-based & auto-calibration
- **[SSD_MIGRATION.md](SSD_MIGRATION.md)** - NVMe SSD setup
- **[RECOVERY.md](RECOVERY.md)** - System recovery

## Root Docs

- **[README.md](../README.md)** - System overview & quick start
- **[SETUP.md](../SETUP.md)** - Complete setup guide

## Key Concepts

### Articulated Cameras
Each camera has independent pan-tilt servos for active perception and object tracking.

### UDM Worldgraph
All sensor data projects into a unified digital twin (TSDF/occupancy). This enables:
- Simulation-independent planning
- Emulating different sensor configurations
- Offline RL training from worldgraph

### Calibration
- **Fixture-based**: Camera intrinsics, kinematic chains, servo zeros
- **Auto-calibration**: Odometry, IMU biases, worldgraph consistency
