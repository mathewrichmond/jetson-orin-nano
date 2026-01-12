# nvblox Processor ROS 2 Package

ROS 2 package for preprocessing RealSense depth data using nvblox-inspired processing for efficient 3D mapping and visualization.

## Features

- **Downsampled Point Clouds**: Reduces point cloud density for efficient visualization
- **TSDF Visualization**: Publishes TSDF (Truncated Signed Distance Function) markers for 3D mapping visualization
- **Mesh Generation**: Placeholder for mesh generation (can be extended with full nvblox integration)
- **Multi-Camera Support**: Processes data from multiple RealSense cameras simultaneously
- **Configurable Processing**: Adjustable voxel size, downsampling factor, and update rates

## Purpose

This node addresses the high drop rate issue when visualizing raw RealSense point cloud data by:
1. Downsampling point clouds before publishing (reduces bandwidth)
2. Processing depth images efficiently
3. Generating TSDF representations for visualization
4. Preparing data for VLA model feature extraction

## Topics

### Subscribed Topics

For each camera (e.g., `camera_front`, `camera_rear`):
- `/hardware/{camera_name}/depth/image_rect_raw` - Depth images (sensor_msgs/Image)
- `/hardware/{camera_name}/color/image_raw` - Color images (sensor_msgs/Image)
- `/hardware/{camera_name}/depth/camera_info` - Camera calibration (sensor_msgs/CameraInfo)

### Published Topics

- `/nvblox/status` - Node status messages (std_msgs/String)

For each camera:
- `/nvblox/{camera_name}/points_downsampled` - Downsampled pointcloud (sensor_msgs/PointCloud2)
- `/nvblox/{camera_name}/mesh` - Mesh markers (visualization_msgs/MarkerArray)
- `/nvblox/{camera_name}/tsdf` - TSDF voxel markers (visualization_msgs/MarkerArray)

## Parameters

- `camera_names` - List of camera names to process (default: ["camera_front", "camera_rear"])
- `voxel_size` - Size of TSDF voxels in meters (default: 0.05)
- `tsdf_truncation_distance` - TSDF truncation distance in meters (default: 0.1)
- `max_tsdf_weight` - Maximum TSDF weight (default: 100.0)
- `mesh_update_rate` - Rate at which mesh/TSDF markers are updated in Hz (default: 5.0)
- `pointcloud_downsample_factor` - Downsampling factor for pointcloud visualization (default: 4)
- `publish_tsdf_markers` - Enable TSDF marker publishing (default: true)
- `publish_mesh_markers` - Enable mesh marker publishing (default: true)
- `publish_downsampled_points` - Enable downsampled pointcloud publishing (default: true)
- `status_topic` - Status topic name (default: "/nvblox/status")
- `namespace` - Namespace for published topics (default: "/nvblox")

## Usage

### Launch with Graph Management

The node is configured in `config/robot/robot_graph.yaml` and should be started using graph management:

```bash
./scripts/system/manage_graph.sh start robot
```

### Manual Launch (for testing)

```bash
ros2 launch nvblox_processor nvblox_processor.launch.py
```

### Launch with Custom Config

```bash
ros2 launch nvblox_processor nvblox_processor.launch.py config_file:=/path/to/config.yaml
```

## Configuration

Configuration files are stored in:
- Centralized: `config/hardware/nvblox_params.yaml`
- Package: `src/hardware_drivers/nvblox_processor/config/nvblox_params.yaml`

## Integration with Visualization

The Foxglove visualization config (`config/visualization/foxglove_all_sensors.json`) has been updated to use the downsampled point clouds from this node instead of raw point clouds, reducing visualization drop rates.

## Future Enhancements

- Full nvblox library integration for GPU-accelerated TSDF processing
- ESDF (Euclidean Signed Distance Field) generation
- Mesh extraction from TSDF volumes
- Multi-frame integration for global mapping
- Feature extraction for VLA model inputs

## Dependencies

- ROS 2 Humble
- cv_bridge
- sensor_msgs
- visualization_msgs
- numpy

## Notes

- The current implementation provides a lightweight processing pipeline
- For full nvblox GPU acceleration, the node can be extended to use the nvblox C++ library
- Downsampling factor of 4 reduces point cloud size by ~16x (4x4 grid)
- TSDF markers are limited to 1000 voxels per update for performance
