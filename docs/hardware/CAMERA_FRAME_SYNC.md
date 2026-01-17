# Camera Frame Synchronization

This document describes camera frame synchronization options for the Isaac robot system.

## Overview

For VLM feature extraction and accurate sensor fusion, camera frames should be synchronized. The Isaac robot system uses:

1. **Hardware Synchronization** (Implemented) - Ground and sync pins connected between cameras
2. **Software Synchronization** (Available as fallback) - Software-based frame synchronization

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

## Hardware Synchronization (RealSense) - IMPLEMENTED

The Isaac robot system uses hardware frame synchronization between the two RealSense cameras:

- **Physical Connection**: Ground and sync pins are connected between the two cameras
- **Inter-camera sync** - One camera (master) generates sync signal, the other (slave) receives it
- **Firmware-controlled** - Configured via pyrealsense2
- **Hardware-level precision** - Provides precise frame synchronization at the hardware level

**This is the production approach** - it uses the cameras' built-in sync mechanism and provides reliable, precise synchronization.

### Firmware-Based Inter-Camera Sync (Recommended)

RealSense cameras support inter-camera synchronization via firmware configuration. This is **preferred over external GPIO triggers** because:
- Uses cameras' built-in sync mechanism
- No external GPIO connections needed
- Cameras handle timing internally
- More reliable than software-only sync

**Hardware Setup (Completed):**
1. **Sync cables connected** between cameras (camera-to-camera)
   - Master camera's sync out → Slave camera's sync in
   - Ground pins connected for common reference
2. **No GPIO connections needed** - sync is handled internally by cameras via firmware

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

1. **For Production:** Hardware frame sync is implemented and active (ground and sync pins connected)
2. **For Development/Fallback:** Software sync available if hardware sync fails
3. **For VLM Features:** Hardware sync provides precise frame alignment for feature extraction
4. **Alternative:** External GPIO sync available if firmware sync not available (not needed with current setup)

## Current Implementation

**Hardware Sync (Production):**
- ✅ Physical sync cables connected between cameras
- ✅ Ground pins connected for common reference
- ✅ Firmware-based inter-camera sync: Implemented and active
- ✅ Automatic master/slave assignment
- ✅ Hardware-level frame synchronization

**Software Sync (Fallback):**
- ✅ Syncs IMU and chassis data to camera frames
- ✅ Software-based multi-camera frame sync (available if hardware sync fails)
- ✅ Configurable sync tolerance

**Future Enhancements:**
- ⏳ nvblox-based fusion (can be added)
- ⏳ RealSense hardware trigger configuration via pyrealsense2
- ⏳ Multi-camera daisy-chain sync support
