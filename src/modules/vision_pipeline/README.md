# Vision Pipeline Module

Visual SLAM, camera calibration, and pipeline orchestration for dual-compute robotics platform.

## Overview

The vision pipeline module provides three nodes for robot vision processing:

1. **Visual SLAM** - RTAB-Map wrapper for global pose estimation
2. **Camera Calibration** - Online camera-IMU extrinsic calibration
3. **Vision Pipeline Orchestrator** - Lifecycle and power management

## Quick Start

### Build

```bash
colcon build --packages-select vision_pipeline --symlink-install
source install/setup.bash
```

### Run Nodes

```bash
# Visual SLAM
ros2 run vision_pipeline visual_slam_node

# Camera Calibration
ros2 run vision_pipeline camera_calibration_node

# Pipeline Orchestrator
ros2 run vision_pipeline vision_pipeline_node
```

## Nodes

### 1. Visual SLAM Node

Wraps RTAB-Map for visual odometry and loop closure detection.

**Key Topics**:
- Input: `/hardware/camera_front/color/image_raw`, depth, camera_info, IMU
- Output: `/vision/global_pose`, `/vision/odometry`, `/vision/trajectory`

**Parameters**: See [visual_slam_node.py](vision_pipeline/visual_slam_node.py)

### 2. Camera Calibration Node

Estimates camera-IMU extrinsic transforms using visual-inertial alignment.

**Key Topics**:
- Input: `/rpi/imu/filtered`, camera info, visual odometry
- Output: `/rpi/calibration/status` (CalibrationStatus)

**Parameters**: See [camera_calibration_node.py](vision_pipeline/camera_calibration_node.py)

### 3. Vision Pipeline Orchestrator

Manages vision system lifecycle, health monitoring, and power modes.

**Modes**:
- **Full**: SLAM + nvblox + calibration (GPU active)
- **Tracking Only**: Localization only (GPU low power)
- **Sleep**: Vision inactive (GPU off)

**Key Topics**:
- Output: `/jetson/health/vision_pipeline`, `/jetson/power/request`

**Parameters**: See [vision_pipeline_node.py](vision_pipeline/vision_pipeline_node.py)

## Testing

```bash
source install/setup.bash
python3 src/modules/vision_pipeline/test/test_node_imports.py
```

Should output: `PHASE 2 VALIDATION: ALL TESTS PASSED ✓`

## Integration

### Dependencies
- Phase 1: Chassis Control (IMU processor, calibration manager)
- Existing: RealSense cameras, nvblox processor
- Optional: rtabmap_ros for SLAM backend

### Install RTAB-Map

```bash
sudo apt install ros-humble-rtabmap-ros
```

## Documentation

- Phase 2 Completion: [/docs/PHASE2_COMPLETION.md](../../docs/PHASE2_COMPLETION.md)
- Architecture Plan: `/home/nano/.claude/plans/swift-rolling-curry.md`

---

**Module Version**: 0.1.0
**Last Updated**: 2026-01-27
**Status**: Ready for RTAB-Map integration and hardware testing
