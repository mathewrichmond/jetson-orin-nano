# Camera Frame Synchronization

This document describes camera frame synchronization options for the Isaac robot system.

## Overview

For VLM feature extraction and accurate sensor fusion, camera frames should be synchronized. There are two approaches:

1. **Hardware Synchronization** (Recommended for production)
2. **Software Synchronization** (Current implementation)

## Software Frame Synchronization

The `sensor_sync` node implements software-based frame synchronization:

- Waits for frames from all cameras within a tolerance window
- Publishes synchronized sensor data when all cameras have frames
- Configurable tolerance: `camera_frame_sync_tolerance` (default: 50ms)

**Configuration:**
```yaml
sensor_sync:
  parameters:
    sync_camera_frames: true
    camera_frame_sync_tolerance: 0.05  # 50ms tolerance
```

**Limitations:**
- Adds latency (waits for slowest camera)
- May drop frames if cameras drift apart
- Not as precise as hardware sync

## Hardware Synchronization (RealSense)

RealSense cameras support firmware-based inter-camera synchronization via:
- **Inter-camera sync** - One camera (master) generates sync signal, others (slaves) receive it
- **Firmware-controlled** - Configured via pyrealsense2, no external GPIO needed
- **Physical sync cables** - Requires sync cables between cameras (camera-to-camera, not GPIO)

**This is the recommended approach** - it uses the cameras' built-in sync mechanism and is more reliable than external GPIO triggers.

### Firmware-Based Inter-Camera Sync (Recommended)

RealSense cameras support inter-camera synchronization via firmware configuration. This is **preferred over external GPIO triggers** because:
- Uses cameras' built-in sync mechanism
- No external GPIO connections needed
- Cameras handle timing internally
- More reliable than software-only sync

**Hardware Setup:**
1. **Connect sync cables** between cameras (camera-to-camera, not to GPIO)
   - Master camera's sync out → Slave camera's sync in
   - Check RealSense documentation for sync pin locations
2. **No GPIO connections needed** - sync is handled internally by cameras

**Software Configuration:**
```yaml
realsense_camera:
  parameters:
    enable_inter_cam_sync: true  # Enable firmware-based sync
    # First camera becomes master (generates sync)
    # Other cameras become slaves (receive sync)
```

**How It Works:**
- First camera is configured as **Master** (mode 1) - generates sync signal
- Other cameras are configured as **Slaves** (mode 2) - receive sync signal
- Cameras synchronize frames internally via firmware
- No external GPIO or software timing needed

**Benefits:**
- ✅ Precise frame synchronization (hardware-level)
- ✅ No external GPIO connections
- ✅ Firmware-controlled (more reliable)
- ✅ Automatic master/slave assignment

### External GPIO Sync (Alternative)

If firmware-based sync is not available, you can use external GPIO triggers:

**GPIO Pin Selection:**
- Pin 16: Motor left direction
- Pin 18: Motor left PWM
- Pin 19: Motor right PWM
- Pin 20: Motor right direction
- **Pin 21: Available for sync signal** ✅

**Hardware Connection:**
1. **Connect GPIO pin 21** from PHAT to camera sync input
2. **Common ground** between PHAT and cameras
3. **Signal level:** 3.3V (Jetson GPIO standard)

**Configuration:**
```yaml
hardware_sync_generator:
  enabled: true
  parameters:
    sync_gpio_pin: 21
    sync_frequency: 15.0  # Match camera FPS
    pulse_width_ms: 1.0
```

**Note:** Firmware-based inter-camera sync is preferred over GPIO triggers.

## nvblox-Based Frame Fusion

As an alternative to frame synchronization, nvblox can be used for:

- **Temporal fusion** - Fuse frames from different times
- **Resampling** - Generate synchronized output at target rate
- **Frame interpolation** - Estimate intermediate frames

### Using nvblox for Frame Sync

The `nvblox_processor` node can be extended to:
1. Buffer frames from multiple cameras
2. Fuse frames temporally using TSDF
3. Generate synchronized output frames

**Configuration:**
```yaml
nvblox_processor:
  parameters:
    enable_frame_fusion: true
    fusion_window: 0.1  # 100ms fusion window
    output_fps: 15.0  # Target output FPS
```

## Recommendations

1. **For Development:** Use software sync (current implementation)
2. **For Production:** Use firmware-based inter-camera sync (preferred) - requires sync cables between cameras
3. **Alternative:** Use external GPIO sync if firmware sync not available
4. **For VLM Features:** Use nvblox fusion for robust temporal alignment

## Current Implementation

**Software Sync:**
- ✅ Syncs IMU and chassis data to camera frames
- ✅ Software-based multi-camera frame sync
- ✅ Configurable sync tolerance

**Hardware Sync:**
- ✅ Firmware-based inter-camera sync: Implemented (requires sync cables between cameras)
- ✅ Automatic master/slave assignment
- ✅ GPIO sync signal generator: Available as alternative
- ⏳ Physical sync cable connections: Required for firmware sync

**Future Enhancements:**
- ⏳ nvblox-based fusion (can be added)
- ⏳ RealSense hardware trigger configuration via pyrealsense2
- ⏳ Multi-camera daisy-chain sync support
