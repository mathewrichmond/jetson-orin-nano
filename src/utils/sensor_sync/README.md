# Sensor Fusion Node

Sensor fusion node that takes high-frequency, time-mismatched raw sensor data and applies post-processing and re-sampling to create fused, time-aligned data. This node is the **single source of truth** for all downstream consumers (VLA feature building, visualization, logging).

## Purpose

The sensor fusion node:
- **Synchronizes** all sensor data to camera frame timestamps
- **Applies post-processing** (Kalman filtering for IMU, downsampling for bandwidth)
- **Re-samples** high-frequency data to uniform rates
- **Fuses** sensor, chassis, and system information into unified outputs
- **Publishes** everything downstream consumers need (except control plan)

## Features

- **Kalman Filtering**: Smooths high-frequency IMU data (reduces noise and bandwidth)
- **Camera Frame Synchronization**: Syncs all sensor data to camera frame timestamps
- **Multi-Camera Frame Sync**: Software-based synchronization of multiple cameras
- **Chassis Data Synchronization**: Syncs battery and status data to camera frames
- **Audio Synchronization**: Syncs audio chunks to camera frames (optional, currently disabled)
- **Configurable Frequency**: Can run at fixed rate or sync to camera frames
- **Single Source of Truth**: Publishes synchronized sensor data for all downstream consumers
- **Bandwidth Reduction**: Filters and resamples all high-frequency sensor data
- **Downsampling**: Provides both feature-quality (moderate) and viz-quality (aggressive) downsampling

## Architecture

```
Raw Sensors → nvblox Processor → Sensor Sync Node → Synchronized Sensor Data (Single Source of Truth)
   (50Hz)      (Full Quality)      (Kalman Filter)         (15Hz, synced)
                                                                    │
                                                                    ├──→ VLA Feature Builder (consumes + control plan, applies history)
                                                                    ├──→ Visualization (consumes viz topics)
                                                                    └──→ Logging (consumes feature topics)
```

**Design Philosophy**: Single data path, no fallbacks
- Raw cameras → nvblox (processes camera data)
- nvblox → sensor_sync (ONLY source for image data)
- Raw camera topics used ONLY for timestamp synchronization

## Topics

### Subscribed (High-Frequency, Time-Mismatched Raw Data)

**Sensor Data:**
- `/phat/imu` - Raw IMU data (high frequency ~50Hz, time-mismatched)
- `/hardware/camera_front/color/image_raw` - Camera timestamps (sync trigger, ~30Hz)
- `/hardware/camera_rear/color/image_raw` - Camera timestamps (sync trigger, ~30Hz)

**Chassis Data:**
- `/irobot/battery` - Raw battery data (high frequency ~10Hz, time-mismatched)
- `/irobot/status` - Raw status messages (time-mismatched)

**Processed Camera/3D Data (from nvblox):**
- `/nvblox/full/camera_*/image` - Processed camera images (full quality)
- `/nvblox/full/camera_*/pointcloud` - Processed pointclouds (full quality)
- `/nvblox/full/mesh` - Processed mesh (full quality)
- `/nvblox/full/tsdf` - Processed TSDF (full quality)

**System Data (optional):**
- `/system/temperature/cpu` - CPU temperature
- `/system/temperature/gpu` - GPU temperature
- `/system/cpu/usage` - CPU usage
- `/system/gpu/usage` - GPU usage
- `/system/memory/usage` - Memory usage
- `/system/disk/usage` - Disk usage
- `/system/power` - Power consumption
- `/system/alerts` - System alerts

**Audio Data (optional):**
- `/microphone/audio` - Raw audio chunks from USB microphone (high frequency, time-mismatched)

### Published (Fused, Time-Aligned, Single Source of Truth)

All outputs are synchronized to camera frame timestamps (~15Hz) and contain everything downstream consumers need.

**Chassis Information:**
- `/sensor_fusion/chassis/battery` - Synchronized battery data (voltage, percentage, current)
- `/sensor_fusion/chassis/status` - Synchronized chassis status messages

**Sensor Information:**
- `/sensor_fusion/imu/filtered` - Kalman-filtered IMU data (linear acceleration, angular velocity)
- `/sensor_fusion/camera_*/color/image_raw` - Feature images (downsampled, ~480×360 @ 15Hz)
- `/sensor_fusion/three_d/*/pointcloud` - Feature pointclouds (downsampled, factor 2)
- `/sensor_fusion/three_d/mesh` - Feature mesh (decimated, 50% reduction)
- `/sensor_fusion/three_d/tsdf` - Feature TSDF (coarser voxels, 0.1m)

**System Information:**
- `/sensor_fusion/system/temperature/cpu` - CPU temperature (synchronized)
- `/sensor_fusion/system/temperature/gpu` - GPU temperature (synchronized)
- `/sensor_fusion/system/cpu/usage` - CPU usage (synchronized)
- `/sensor_fusion/system/gpu/usage` - GPU usage (synchronized)
- `/sensor_fusion/system/memory/usage` - Memory usage (synchronized)
- `/sensor_fusion/system/disk/usage` - Disk usage (synchronized)
- `/sensor_fusion/system/power` - Power consumption (synchronized)
- `/sensor_fusion/system/alerts` - System alerts (synchronized)
- `/sensor_fusion/system/hardware/camera_sync_status` - Camera synchronization status

**Audio Information (optional):**
- `/sensor_fusion/audio/raw` - Synchronized raw audio chunks (aligned to camera frames, ~15Hz)

**Visualization Topics** (aggressive downsampling, for remote visualization):
- `/viz/remote/camera_*/color/image_raw` - Visualization images (downsampled, ~320×240 @ 5Hz)
- `/viz/remote/imu/filtered` - Visualization IMU
- `/viz/remote/chassis/battery` - Visualization battery
- `/viz/remote/system/*` - Visualization system status
- `/viz/remote/three_d/*` - Visualization 3D data (heavily downsampled)
- `/viz/remote/audio/stats` - Audio statistics (max volume per channel, synchronized to camera frames)

**Note**: These topics contain **all information** downstream consumers need (except control plan):
- **VLA feature building node**: Consumes feature topics + control plan, applies history buffering
- **Visualization**: Consumes viz topics for remote monitoring
- **Logging**: Consumes feature topics for data recording

## Configuration

Key parameters:
- `sync_to_camera`: `true` - Sync to camera frames (recommended)
- `sync_camera_frames`: `true` - Sync multiple cameras together (software sync)
- `camera_frame_sync_tolerance`: `0.05` - Max time diff between cameras (50ms)
- `target_frequency`: `15.0` - Target frequency (matches camera FPS)
- `imu_filter_enabled`: `true` - Enable Kalman filtering
- `imu_process_noise`: `0.01` - Kalman filter process noise
- `imu_measurement_noise`: `0.1` - Kalman filter measurement noise

**Audio Synchronization Parameters (optional):**
- `enable_audio_sync`: `true` - Enable audio synchronization (enabled by default)
- `audio_topic`: `/microphone/audio` - Input audio topic from USB microphone
- `audio_sample_rate`: `16000` - Audio sample rate (Hz)
- `audio_channels`: `2` - Number of audio channels (stereo)
- `audio_format`: `S16_LE` - Audio format (16-bit signed little-endian)
- `audio_sync_tolerance`: `0.1` - Time tolerance for audio chunk synchronization (seconds)

**Note on Audio Synchronization**: Audio synchronization synchronizes audio chunks to camera frame timestamps, resamples/concatenates chunks to match frame duration, and publishes both raw synchronized audio for features (`/sensor_fusion/audio/raw`) and audio statistics for visualization (`/viz/remote/audio/stats`). The audio processing includes error handling to gracefully handle missing or malformed audio data.

## Usage

The node is automatically included in the robot graph. It:

1. **Subscribes** to high-frequency, time-mismatched raw sensor data:
   - IMU (~50Hz)
   - Chassis data (~10Hz)
   - Camera timestamps (~30Hz, for sync)
   - nvblox-processed camera/3D data (full quality)
   - System status (temperatures, CPU/GPU, etc.)
   - Audio chunks (if enabled, ~10Hz from USB microphone)

2. **Applies post-processing**:
   - Kalman filtering for IMU (reduces noise)
   - Image downsampling (feature and viz quality)
   - 3D data downsampling (pointclouds, mesh, TSDF)
   - Audio chunk resampling/concatenation (if enabled, aligns audio to camera frames)

3. **Re-samples** high-frequency data to uniform camera frame rate (~15Hz)

4. **Synchronizes** all data to camera frame timestamps (time-aligned)

5. **Fuses** sensor, chassis, and system information into unified outputs

6. **Publishes** fused, time-aligned data as the **single source of truth** for all downstream consumers

This reduces sensor data bandwidth by ~70% while maintaining synchronization. All downstream consumers use these synchronized topics (except control plan, which comes from the planner).

## Downstream Consumers

- **VLA Feature Building Node**: Consumes `/sensor_fusion/*` feature topics + control plan, applies history buffering, builds VLA features
- **Visualization**: Consumes `/viz/remote/*` topics for remote monitoring
- **Logging**: Consumes `/sensor_fusion/*` feature topics for data recording

## Camera Frame Synchronization

The node supports software-based multi-camera frame synchronization:
- Waits for frames from all cameras within tolerance window
- Publishes synchronized data when all cameras are ready
- Configurable tolerance (default: 50ms)

For hardware-based camera sync, see [Camera Frame Sync Documentation](../../docs/hardware/CAMERA_FRAME_SYNC.md).
