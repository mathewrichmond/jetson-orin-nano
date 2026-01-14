# Foxglove Studio Layouts

This document describes the available Foxglove Studio visualization layouts for the Isaac robot system.

## Available Layouts

### 1. `sensor_bridged.json` - Optimized for Remote/Bridged Connection

**Purpose**: Low-bandwidth visualization for remote monitoring via Foxglove Bridge

**Topics Used**:
- `/viz/remote/camera_front/color/image_raw` - Downsampled camera (320×240 @ 10 Hz)
- `/viz/remote/imu/filtered` - Filtered IMU data
- `/viz/remote/chassis/battery` - Synced battery data
- `/viz/remote/3d/mesh` - Decimated fused mesh (75% reduction @ 5 Hz)
- `/viz/remote/3d/pointcloud` - Downsampled pointcloud (factor 16 @ 5 Hz)
- `/system/*` - System monitoring

**Features**:
- Front camera only (reduces bandwidth)
- Aggressively downsampled 3D data
- Essential sensors only
- Optimized for 5 MB/s bridge budget

**Use Case**: Remote monitoring, debugging over network

### 2. `sensor_raw.json` - Full Data for Direct Connection

**Purpose**: Full-quality visualization for direct ROS 2 connection

**Topics Used**:
- `/hardware/camera_*/color/image_raw` - Raw camera images (640×480 @ 30 Hz)
- `/phat/imu` - Raw IMU data
- `/sensor_fusion/imu/filtered` - Filtered IMU data (for comparison)
- `/irobot/battery` - Raw battery data
- `/sensor_fusion/chassis/battery` - Synced battery data (for comparison)
- `/nvblox/full/camera_*/pointcloud` - Full quality pointclouds
- `/nvblox/full/mesh` - Full quality fused mesh
- `/nvblox/full/tsdf` - Full quality TSDF
- `/microphone/audio` - Audio data

**Features**:
- Both cameras (front + rear)
- Raw and filtered/synced sensor comparison
- Full quality 3D data (per-camera and fused)
- All sensor data

**Use Case**: Local development, debugging, sensor comparison

## Usage

### Loading a Layout in Foxglove Studio

1. Open Foxglove Studio
2. Connect to your data source (Bridge or direct ROS 2 connection)
3. Go to **Layouts** → **Import**
4. Select the appropriate layout file:
   - `config/visualization/sensor_bridged.json` (for bridge)
   - `config/visualization/sensor_raw.json` (for direct connection)

### Choosing the Right Layout

1. **For Remote Monitoring:** Use `sensor_bridged.json` with Foxglove Bridge
2. **For Local Development:** Use `sensor_raw.json` with direct ROS 2 connection
3. **For Debugging:** Use `sensor_raw.json` to compare raw vs synced data

## Topic Structure

### Bridged Topics (`/viz/remote/*`)
- Aggressively downsampled for low bandwidth
- Single camera (front only)
- Fused 3D data only
- Essential sensors

### Raw Topics (`/hardware/*`, `/nvblox/full/*`)
- Full quality, no downsampling
- All cameras
- Per-camera and fused 3D data
- All sensors

### Feature Topics (`/sensor_fusion/*`)
- Moderate downsampling for VLM features
- Used by feature builder, not visualization layouts

## Bandwidth Considerations

- **Bridged Layout**: ~3.2 MB/s (within 5 MB/s bridge budget)
- **Raw Layout**: ~30 MB/s (requires direct connection)

## Updating Layouts

Layouts are JSON files that can be edited directly or modified through Foxglove Studio's UI. After making changes, export the layout and save it to `config/visualization/`.
