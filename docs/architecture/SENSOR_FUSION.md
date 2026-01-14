# Sensor Fusion Architecture

This document describes the sensor fusion architecture, bandwidth planning, and downsampling pipeline for the Isaac robot system.

## Philosophy

**Compute Once, Downsample for Consumers**

Rather than choosing one representation over another, we:
1. **Compute everything** at full quality/resolution
2. **Downsample/decimate** to fit bandwidth budgets for different consumers
3. **Preserve quality** where bandwidth allows, reduce where needed

This approach provides maximum flexibility, preserves quality, and enables precise bandwidth control.

## Architecture Overview

```
Raw Sensors (High Frequency)
├── IMU (50 Hz)
├── Battery (10 Hz)
├── Status (10 Hz)
└── Cameras (30 Hz)
    ↓
nvblox Processor (Full Quality Computation)
├── Full Resolution Pointclouds
├── Full Quality Mesh
├── Complete TSDF (fused)
└── Full Resolution Images
    ↓
Fusion Node (Synchronization + Downsampling)
├──→ Feature Topics (15 Hz, moderate downsampling)
│   ├── Images (480×360)
│   ├── Pointclouds (factor 2)
│   ├── Mesh (50% decimation)
│   └── TSDF (0.1m voxels)
│
└──→ Viz Topics (5-10 Hz, aggressive downsampling)
    ├── Images (320×240 @ 10 Hz)
    ├── Pointclouds (factor 16 @ 5 Hz)
    ├── Mesh (75% decimation @ 5 Hz)
    └── TSDF Mesh (extracted @ 2 Hz)
```

## Bandwidth Budgets

### Total System Bandwidth
- **Jetson Orin Nano**: Limited by USB 3.0, network, and processing
- **Target**: Stay under 50 MB/s total system bandwidth

### Allocation
1. **Feature Builder Budget**: 20 MB/s (for VLM input features)
2. **Bridge Budget**: 5 MB/s (for remote visualization)
3. **Direct Connection Budget**: 30 MB/s (for local development)

## Raw Sensor Frequencies

| Sensor | Topic | Frequency | Message Size | Bandwidth |
|--------|-------|-----------|--------------|-----------|
| IMU | `/phat/imu` | 50 Hz | 200 B | 0.01 MB/s |
| Battery | `/irobot/battery` | 10 Hz | 100 B | 0.001 MB/s |
| Status | `/irobot/status` | 10 Hz | 50 B | 0.0005 MB/s |
| Camera Front | `/hardware/camera_front/color/image_raw` | 30 Hz | 921 KB | 27.6 MB/s |
| Camera Rear | `/hardware/camera_rear/color/image_raw` | 30 Hz | 921 KB | 27.6 MB/s |
| System Stats | `/system/.*` | 5 Hz | 200 B | 0.001 MB/s |

**Total Raw Sensors**: ~55.2 MB/s (mostly cameras)

## Full Quality Computation (nvblox)

nvblox processes all depth data at full quality and outputs to `/nvblox/full/*` topics.

### nvblox Outputs (Full Quality)

| Output | Quality | Frequency | Size | Purpose |
|--------|---------|-----------|------|---------|
| Pointclouds | Full resolution | 15 Hz | ~3.7 MB/frame | Base for all downsampling |
| Mesh | Full quality | 5 Hz | ~500 KB-2 MB/frame | Base for decimation |
| TSDF | Complete voxel grid (0.05m) | 5 Hz | ~4.8 MB/frame | Base for extraction |
| Images | Full resolution (640×480) | 15 Hz | 921 KB/frame | Base for resizing |

**Total Full Quality**: ~60-70 MB/s (not bridged, internal only)

### Multi-Camera Fusion

nvblox can fuse multiple camera views into a single unified TSDF:
- **Fused TSDF**: Single voxel grid combining front + rear cameras
- **Fused Mesh**: Unified mesh from fused TSDF
- **Benefits**: Single unified 3D map, better spatial understanding, reduced bandwidth

**Configuration:**
```yaml
nvblox_processor:
  parameters:
    fuse_cameras: true  # Fuse front + rear into unified TSDF/mesh
```

## Fusion Node Outputs

The fusion node synchronizes all sensor data to camera frames and downsamples for different consumers.

### Feature Topics (20 MB/s Budget)

**Strategy**: Moderate downsampling to preserve quality while fitting budget

| Topic | Full Quality | Downsampled | Frequency | Bandwidth | Purpose |
|-------|--------------|-------------|-----------|-----------|---------|
| `/sensor_fusion/imu/filtered` | - | - | 15 Hz | 0.003 MB/s | VLM features |
| `/sensor_fusion/chassis/battery` | - | - | 15 Hz | 0.0015 MB/s | VLM features |
| `/sensor_fusion/chassis/status` | - | - | 15 Hz | 0.0008 MB/s | VLM features |
| `/sensor_fusion/camera_front/color/image_raw` | 640×480 | 480×360 | 15 Hz | 7.8 MB/s | VLM features |
| `/sensor_fusion/camera_rear/color/image_raw` | 640×480 | 480×360 | 15 Hz | 7.8 MB/s | VLM features |
| `/sensor_fusion/3d/camera_front/pointcloud` | Full | Factor 2 | 15 Hz | 1.4 MB/s | 3D geometry |
| `/sensor_fusion/3d/camera_rear/pointcloud` | Full | Factor 2 | 15 Hz | 1.4 MB/s | 3D geometry |
| `/sensor_fusion/3d/mesh` | Full | 50% decimation | 5 Hz | 1.0 MB/s | Fused mesh |
| `/sensor_fusion/3d/tsdf` | 0.05m voxels | 0.1m voxels | 5 Hz | 0.6 MB/s | TSDF grid |
| `/sensor_fusion/vlm_features` | - | - | 15 Hz | 0.15 MB/s | Pre-processed |

**Total Feature Builder**: ~20.2 MB/s (within 20 MB/s budget ✅)

### Visualization Topics

#### Raw Visualization (Direct Connection - 30 MB/s Budget)

| Topic | Frequency | Message Size | Bandwidth | Purpose |
|-------|-----------|--------------|-----------|---------|
| `/sensor_fusion/imu/filtered` | 15 Hz | 200 B | 0.003 MB/s | Debug |
| `/sensor_fusion/chassis/battery` | 15 Hz | 100 B | 0.0015 MB/s | Monitoring |
| `/hardware/camera_front/color/image_raw` | 15 Hz | 921 KB | 13.8 MB/s | Visualization |
| `/hardware/camera_rear/color/image_raw` | 15 Hz | 921 KB | 13.8 MB/s | Visualization |
| `/system/.*` | 5 Hz | 200 B | 0.001 MB/s | System monitoring |

**Total Raw Viz**: ~27.6 MB/s (within 30 MB/s direct connection budget)

#### Remote Visualization (Bridge - 5 MB/s Budget)

**Strategy**: Aggressive downsampling to fit bridge bandwidth

| Topic | Full Quality | Downsampled | Frequency | Bandwidth | Purpose |
|-------|--------------|-------------|-----------|-----------|---------|
| `/viz/remote/imu/filtered` | - | - | 15 Hz | 0.003 MB/s | Monitoring |
| `/viz/remote/chassis/battery` | - | - | 15 Hz | 0.0015 MB/s | Monitoring |
| `/viz/remote/camera_front/color/image_raw` | 640×480 @ 15 Hz | 320×240 @ 10 Hz | 10 Hz | 2.3 MB/s | Visualization |
| `/viz/remote/3d/pointcloud` | Full @ 15 Hz | Factor 16 @ 5 Hz | 5 Hz | 0.2 MB/s | 3D pointcloud |
| `/viz/remote/3d/mesh` | Full @ 5 Hz | 75% decimation | 5 Hz | 0.5 MB/s | 3D mesh |
| `/viz/remote/3d/tsdf_mesh` | Full TSDF | Mesh extraction | 2 Hz | 0.2 MB/s | TSDF surface |
| `/system/.*` | - | - | 5 Hz | 0.001 MB/s | System monitoring |

**Total Remote Viz**: ~3.2 MB/s (within 5 MB/s bridge budget ✅)

## Downsampling Strategies

### Image Downsampling

**Feature Topics:**
- Resolution: 480×360 (from 640×480) - 56% pixels
- Method: Bilinear interpolation
- Quality: High (preserve detail)
- Bandwidth: 7.8 MB/s per camera

**Viz Topics:**
- Resolution: 320×240 (from 640×480) - 25% pixels
- Frequency: 10 Hz (from 15 Hz) - 33% reduction
- Method: Bilinear interpolation
- Quality: Medium (acceptable for visualization)
- Bandwidth: 2.3 MB/s per camera

### Pointcloud Downsampling

**Feature Topics:**
- Factor: 2× (every 2nd point) - 75% reduction
- Method: Uniform grid downsampling
- Preserve: Color information if available
- Bandwidth: 1.4 MB/s per camera

**Viz Topics:**
- Factor: 16× (every 16th point) - 94% reduction
- Frequency: 5 Hz (from 15 Hz) - 67% reduction
- Method: Voxel grid downsampling (0.1m voxels)
- Preserve: Structure, discard fine details
- Bandwidth: 0.2 MB/s (fused, single pointcloud)

### Mesh Decimation

**Feature Topics:**
- Reduction: 50% triangles
- Method: Quadric error metric (QEM) decimation
- Preserve: Important geometric features
- Frequency: 5 Hz
- Bandwidth: 1.0 MB/s (fused mesh)

**Viz Topics:**
- Reduction: 75% triangles
- Method: Aggressive decimation
- Preserve: Overall shape, smooth surfaces
- Frequency: 5 Hz
- Bandwidth: 0.5 MB/s (fused mesh)

### TSDF Processing

**Feature Topics:**
- Voxel size: 0.1m (from 0.05m) - 87.5% reduction
- Method: Coarser voxel grid
- Preserve: Full TSDF structure
- Frequency: 5 Hz
- Bandwidth: 0.6 MB/s

**Viz Topics:**
- Method: Extract mesh from TSDF (marching cubes)
- Frequency: 2 Hz (from 5 Hz) - 60% reduction
- Preserve: Surface representation only
- Bandwidth: 0.2 MB/s

## Topic Naming Convention

### Raw Sensors
- `/phat/imu` - Raw IMU (50 Hz)
- `/irobot/battery` - Raw battery (10 Hz)
- `/irobot/status` - Raw status (10 Hz)
- `/hardware/camera_*/color/image_raw` - Raw camera images (30 Hz)

### Full Quality (nvblox Outputs)
- `/nvblox/full/camera_*/pointcloud` - Full resolution pointclouds
- `/nvblox/full/mesh` - Full quality fused mesh
- `/nvblox/full/tsdf` - Complete TSDF voxel grid
- `/nvblox/full/camera_*/image` - Full resolution images

### Feature Topics (Fusion Node Output)
- `/sensor_fusion/imu/filtered` - Filtered IMU (15 Hz)
- `/sensor_fusion/chassis/battery` - Synced battery (15 Hz)
- `/sensor_fusion/chassis/status` - Synced status (15 Hz)
- `/sensor_fusion/camera_*/color/image_raw` - Synced camera images (480×360, 15 Hz)
- `/sensor_fusion/3d/camera_*/pointcloud` - Downsampled pointclouds (factor 2, 15 Hz)
- `/sensor_fusion/3d/mesh` - Decimated mesh (50%, 5 Hz)
- `/sensor_fusion/3d/tsdf` - Coarser TSDF (0.1m voxels, 5 Hz)
- `/sensor_fusion/vlm_features` - Pre-processed VLM features (15 Hz)

### Visualization Topics (Fusion Node Output)
- `/viz/remote/imu/filtered` - Filtered IMU for bridge (15 Hz)
- `/viz/remote/chassis/battery` - Battery for bridge (15 Hz)
- `/viz/remote/camera_front/color/image_raw` - Low-res camera (320×240, 10 Hz)
- `/viz/remote/3d/pointcloud` - Lightweight pointcloud (factor 16, 5 Hz)
- `/viz/remote/3d/mesh` - Decimated mesh (75%, 5 Hz)
- `/viz/remote/3d/tsdf_mesh` - TSDF mesh extraction (2 Hz)

## Downsampling Algorithms

### Image Resizing
- Use OpenCV `cv2.resize()` with bilinear interpolation
- Maintain aspect ratio
- Preserve color information

### Pointcloud Downsampling
- **Uniform grid**: Every Nth point (simple, fast)
- **Voxel grid**: Points within same voxel → single point (preserves structure)
- **Random sampling**: For very aggressive downsampling

### Mesh Decimation
- **Quadric error metric (QEM)**: Preserves important features
- **Edge collapse**: Iteratively collapse edges with lowest error
- **Preserve**: Boundaries, sharp features, color

### TSDF Processing
- **Coarser voxel grid**: Increase voxel size (2× = 87.5% reduction)
- **Mesh extraction**: Marching cubes algorithm from TSDF
- **Sparse representation**: Only active voxels (further reduction)

## Bandwidth Calculation

### Formula
```
Bandwidth (MB/s) = (Message Size × Frequency) / 1,000,000
```

### Message Size Estimates

#### Camera Topics
- **640×480 RGB**: ~921 KB per frame (640 × 480 × 3 bytes)
- **480×360 RGB**: ~518 KB per frame
- **320×240 RGB**: ~230 KB per frame
- **Camera Info**: ~1 KB per frame

#### 3D Data Topics
- **Full Pointcloud** (640×480): ~3.7 MB per frame (307,200 points × 12 bytes)
- **Factor 2 Pointcloud**: ~1.4 MB per frame
- **Factor 16 Pointcloud**: ~230 KB per frame
- **Full Mesh**: ~500 KB-2 MB per frame (depends on complexity)
- **50% Decimated Mesh**: ~250 KB-1 MB per frame
- **75% Decimated Mesh**: ~125 KB-500 KB per frame
- **Full TSDF** (0.05m): ~4.8 MB per frame (600,000 voxels × 8 bytes)
- **Coarse TSDF** (0.1m): ~600 KB per frame

#### Sensor Topics
- **IMU** (`sensor_msgs/Imu`): ~200 bytes per message
- **Battery State**: ~100 bytes per message
- **Status String**: ~50 bytes per message
- **System Stats**: ~50-200 bytes per message

## Graph Configurations

### `robot` Graph (Full System)
- All sensors enabled
- nvblox processor enabled (full quality)
- Fusion node enabled
- Feature builder enabled
- Bridge enabled (remote visualization)
- Visualization topics enabled

**Use Case**: Development, testing, remote monitoring

**Bandwidth**: ~30 MB/s total (feature + bridge)

### `minimal` Graph (Production)
- All sensors enabled
- nvblox processor enabled (full quality)
- Fusion node enabled
- Feature builder enabled
- Bridge **disabled**
- Visualization topics **disabled**

**Use Case**: Autonomous operation, production deployment

**Bandwidth**: ~20 MB/s total (feature only)

## Configuration

### nvblox Processor

```yaml
nvblox_processor:
  parameters:
    camera_names: ["camera_front", "camera_rear"]
    voxel_size: 0.05  # Fine voxels for full quality TSDF
    fuse_cameras: true  # Fuse front + rear into unified TSDF/mesh
    pointcloud_downsample_factor: 1  # Full resolution
    publish_tsdf_markers: true  # Full TSDF
    publish_mesh_markers: true  # Full quality mesh
    namespace: "/nvblox/full"  # Full quality namespace
```

### Fusion Node

```yaml
sensor_fusion:
  parameters:
    # Synchronization
    sync_to_camera: true
    sync_camera_frames: true
    target_frequency: 15.0  # Match camera FPS for features

    # Feature topic downsampling (moderate)
    feature_image_width: 480
    feature_image_height: 360
    feature_pointcloud_factor: 2
    feature_mesh_decimation: 0.5  # 50% reduction
    feature_tsdf_voxel_size: 0.1  # Coarser voxels

    # Viz topic downsampling (aggressive)
    publish_viz_topics: true
    viz_image_width: 320
    viz_image_height: 240
    viz_image_fps: 10
    viz_pointcloud_factor: 16
    viz_mesh_decimation: 0.75  # 75% reduction
    viz_tsdf_extract_mesh: true
    viz_tsdf_fps: 2
```

## Bandwidth Monitoring

### Check Actual Bandwidth
```bash
# Monitor network traffic
sudo iftop -i wlan0

# Check topic rates
ros2 topic hz /sensor_fusion/camera_front/color/image_raw
ros2 topic hz /sensor_fusion/3d/camera_front/pointcloud

# Estimate bandwidth
ros2 topic bw /sensor_fusion/camera_front/color/image_raw
ros2 topic bw /sensor_fusion/3d/mesh
```

### Bandwidth Alerts
- Feature builder exceeds 20 MB/s → Reduce camera resolution or FPS
- Bridge exceeds 5 MB/s → Reduce visualization resolution or disable cameras
- Total system exceeds 50 MB/s → Reduce sensor frequencies or disable non-essential sensors

### Quality Metrics
- Pointcloud density (points per m³)
- Mesh triangle count
- Image resolution
- TSDF voxel resolution

## Benefits

1. **Quality Preservation**: Full quality computation preserves all information
2. **Flexibility**: Can adjust downsampling without recomputing
3. **Efficiency**: Compute once, serve many consumers
4. **Bandwidth Control**: Precise control over bandwidth usage
5. **Future-Proof**: Easy to add new consumers with different quality needs
6. **All Representations**: Image, pointcloud, mesh, and TSDF all available

## Implementation Status

### Completed
- ✅ Bandwidth planning and allocation
- ✅ Downsampling pipeline architecture design
- ✅ Graph configurations (robot, minimal)
- ✅ Topic naming conventions

### In Progress
- ⏳ Rename sensor_sync → sensor_fusion (package/node names)
- ⏳ Implement downsampling in fusion node
- ⏳ Update nvblox to output full quality to `/nvblox/full/*`
- ⏳ Add multi-camera fusion in nvblox

### Future Enhancements

1. **Adaptive Downsampling**: Adjust based on network conditions
2. **Quality Presets**: Predefined quality levels (low/medium/high)
3. **Selective Downsampling**: Downsample some data types more than others
4. **Compression**: Add compression (JPEG, pointcloud compression) before downsampling
5. **Bandwidth Monitoring**: Automatic adjustment based on actual usage

## Recommendations

1. **For Features**: Use moderate downsampling (preserve quality for VLM)
2. **For Visualization**: Use aggressive downsampling (fit bridge budget)
3. **Pre-Fuse Cameras**: Yes for visualization (single unified mesh), optional for features
4. **Monitor Bandwidth**: Track actual usage and adjust downsampling accordingly
5. **Use All Representations**: Image, pointcloud, mesh, and TSDF provide complementary information
