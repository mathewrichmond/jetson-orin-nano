# Sensor Synchronization and Filtering

Sensor synchronization node that synchronizes sensor data to camera frames, applies Kalman filtering to IMU data, and prepares synchronized sensor data for VLM feature extraction.

## Features

- **Kalman Filtering**: Smooths high-frequency IMU data (reduces noise and bandwidth)
- **Camera Frame Synchronization**: Syncs all sensor data to camera frame timestamps
- **Multi-Camera Frame Sync**: Software-based synchronization of multiple cameras
- **Chassis Data Synchronization**: Syncs battery and status data to camera frames
- **Configurable Frequency**: Can run at fixed rate or sync to camera frames
- **VLM Feature Preparation**: Publishes synchronized sensor data ready for VLM input
- **Bandwidth Reduction**: Filters and resamples all high-frequency sensor data

## Architecture

```
Raw Sensors → Sensor Sync Node → Filtered/Synchronized Sensors → Bridge/VLM
   (50Hz)         (Kalman Filter)         (15Hz, synced)          (lower bandwidth)
```

## Topics

### Subscribed
- `/phat/imu` - Raw IMU data (high frequency ~50Hz)
- `/irobot/battery` - Raw battery data (high frequency ~10Hz)
- `/irobot/status` - Raw status messages
- `/hardware/camera_front/color/image_raw` - Camera frames (sync trigger)
- `/hardware/camera_rear/color/image_raw` - Camera frames (sync trigger)

### Published
- `/sensor_sync/imu/filtered` - Kalman-filtered IMU data (synced to camera frames, ~15Hz)
- `/sensor_sync/chassis/battery` - Synchronized battery data (synced to camera frames)
- `/sensor_sync/chassis/status` - Synchronized status messages (synced to camera frames)
- `/sensor_sync/vlm_features` - Synchronized sensor data for VLM (JSON format)
- `/sensor_sync/status` - Node status

## Configuration

Key parameters:
- `sync_to_camera`: `true` - Sync to camera frames (recommended)
- `sync_camera_frames`: `true` - Sync multiple cameras together (software sync)
- `camera_frame_sync_tolerance`: `0.05` - Max time diff between cameras (50ms)
- `target_frequency`: `15.0` - Target frequency (matches camera FPS)
- `imu_filter_enabled`: `true` - Enable Kalman filtering
- `imu_process_noise`: `0.01` - Kalman filter process noise
- `imu_measurement_noise`: `0.1` - Kalman filter measurement noise

## Usage

The node is automatically included in the robot graph. It:
1. Subscribes to raw high-frequency sensor data (IMU ~50Hz, chassis ~10Hz)
2. Applies Kalman filtering to smooth IMU data
3. Synchronizes all sensor data to camera frame timestamps
4. Optionally waits for all cameras to have frames (multi-camera sync)
5. Publishes synchronized data at camera frame rate (~15Hz)

This reduces sensor data bandwidth by ~70% while maintaining synchronization with camera frames for VLM feature extraction.

## Camera Frame Synchronization

The node supports software-based multi-camera frame synchronization:
- Waits for frames from all cameras within tolerance window
- Publishes synchronized data when all cameras are ready
- Configurable tolerance (default: 50ms)

For hardware-based camera sync, see [Camera Frame Sync Documentation](../../docs/hardware/CAMERA_FRAME_SYNC.md).
