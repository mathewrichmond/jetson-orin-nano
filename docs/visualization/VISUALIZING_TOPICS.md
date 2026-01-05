# Visualizing Arbitrary Topics

This guide shows how to visualize any ROS 2 topics, including processed, fused, or custom data streams.

## Overview

The visualization system is **topic-agnostic** - it can visualize any ROS 2 topics regardless of their source. This means you can:

- Visualize raw camera feeds
- Visualize processed/fused video streams
- Visualize pointclouds from any source
- Visualize custom sensor data
- Visualize system metrics
- Visualize any combination of topics

## Quick Start

### 1. Launch Visualization (Generic)

```bash
# Web-based (works on Mac/Windows/Ubuntu)
ros2 launch isaac_robot rosbridge.launch.py

# Or unified visualization
ros2 launch isaac_robot unified_visualization.launch.py viz_backend:=rosbridge
```

### 2. Connect Visualization Client

**Foxglove Studio**:
- Open Foxglove Studio
- Connect → ROS 2 → `ws://<robot-ip>:9090`
- You'll see all available topics automatically

**Web Browser**:
- Open [studio.foxglove.dev](https://studio.foxglove.dev)
- Connect to `ws://<robot-ip>:9090`

### 3. Add Topics to Visualization

Once connected, you can add **any topics** to your visualization:

1. **In Foxglove Studio**:
   - Click "+" to add panels
   - Select panel type (Image, PointCloud, Plot, etc.)
   - Choose any topic from the dropdown
   - Configure as needed

2. **Topic Discovery**:
   ```bash
   # List all available topics
   ros2 topic list

   # See topic type
   ros2 topic type /your/topic/name

   # Echo topic to verify it's publishing
   ros2 topic echo /your/topic/name
   ```

## Common Visualization Scenarios

### Visualizing Processed Video Streams

If you have a processing node that publishes to `/processed/video`:

1. **In Foxglove Studio**:
   - Add Image panel
   - Select topic: `/processed/video`
   - Configure camera info if available

2. **Example topics**:
   - `/fused/stereo/image` - Fused stereo images
   - `/processed/depth/image` - Processed depth
   - `/detection/annotated/image` - Detection results
   - `/segmentation/image` - Segmentation results

### Visualizing Fused Pointclouds

If you have a fusion node publishing to `/fused/pointcloud`:

1. **In Foxglove Studio**:
   - Add 3D panel
   - Select topic: `/fused/pointcloud`
   - Configure color/display options

2. **Example topics**:
   - `/fused/pointcloud` - Multi-camera fused pointcloud
   - `/processed/pointcloud` - Processed pointcloud
   - `/obstacles/pointcloud` - Detected obstacles
   - `/ground/pointcloud` - Ground plane extraction

### Visualizing Custom Data

**Plots** (for numeric data):
- `/system/temperature/cpu` - CPU temperature
- `/control/velocity` - Control velocity
- `/sensor/imu/acceleration` - IMU acceleration

**Raw Messages** (for debugging):
- `/realsense/status` - Camera status
- `/system/alerts` - System alerts
- `/control/commands` - Control commands

**Images** (for any image topic):
- `/camera_front/color/image_raw` - Raw camera feed
- `/processed/image` - Processed image
- `/visualization/debug` - Debug visualization

## Creating Custom Layouts

### Foxglove Studio Layouts

1. **Arrange panels** as desired
2. **Add topics** to each panel
3. **Configure** panel settings
4. **Save layout**: File → Export Layout
5. **Share**: Save JSON file for reuse

### Example Layout Structure

```json
{
  "layout": {
    "name": "Custom Robot Visualization",
    "root": {
      "direction": "row",
      "first": {
        "type": "Image",
        "config": {
          "topic": "/your/processed/image/topic"
        }
      },
      "second": {
        "type": "Plot",
        "config": {
          "paths": [
            {"value": "/your/metric/topic"}
          ]
        }
      }
    }
  }
}
```

## RViz2 Configuration

For RViz2, you can create custom configs:

1. **Launch RViz2**:
   ```bash
   ros2 launch isaac_robot unified_visualization.launch.py viz_backend:=rviz2
   ```

2. **Add displays**:
   - Click "Add" → Select display type
   - Choose topic from dropdown
   - Configure settings

3. **Save config**: File → Save Config As

4. **Load config**:
   ```bash
   ros2 launch isaac_robot unified_visualization.launch.py \
     viz_backend:=rviz2 \
     rviz_config:=/path/to/your/config.rviz
   ```

## Topic Remapping

If your topics use different names, you can remap them:

```bash
# Remap topic for visualization
ros2 run your_package your_node \
  --ros-args \
  -r /original/topic:=/visualization/topic
```

Or use topic remapping in launch files.

## Best Practices

1. **Separate visualization from data sources**:
   - Launch data nodes separately
   - Launch visualization separately
   - This allows flexible topic selection

2. **Use descriptive topic names**:
   - `/processed/image` instead of `/img`
   - `/fused/pointcloud` instead of `/pc`

3. **Publish camera info** for image topics:
   - Helps with proper visualization
   - Enables camera overlays

4. **Use standard message types**:
   - `sensor_msgs/Image` for images
   - `sensor_msgs/PointCloud2` for pointclouds
   - `std_msgs/Float64` for numeric data

5. **Organize topics with namespaces**:
   - `/camera_front/...` - Front camera topics
   - `/processed/...` - Processed data
   - `/control/...` - Control topics

## Troubleshooting

### Topic Not Appearing

1. **Check topic exists**:
   ```bash
   ros2 topic list | grep your_topic
   ```

2. **Check topic is publishing**:
   ```bash
   ros2 topic hz /your/topic
   ```

3. **Check topic type**:
   ```bash
   ros2 topic type /your/topic
   ```

### Visualization Not Updating

1. **Check topic frequency**: Low frequency topics update slowly
2. **Check network**: For remote visualization, check connectivity
3. **Check rosbridge**: Verify rosbridge is running and connected

### Performance Issues

1. **Reduce image resolution** for high-frequency topics
2. **Use compression** for image topics
3. **Filter topics** - only visualize what you need
4. **Use lower update rates** for non-critical visualizations

## Examples

### Example 1: Visualize Processed Video + System Metrics

```bash
# Terminal 1: Launch your processing nodes
ros2 run your_package video_processor

# Terminal 2: Launch visualization
ros2 launch isaac_robot rosbridge.launch.py

# Terminal 3: Connect Foxglove Studio
# Add Image panel → /processed/video
# Add Plot panel → /system/cpu/usage
```

### Example 2: Visualize Fused Pointcloud + Annotations

```bash
# Launch fusion node
ros2 run your_package pointcloud_fusion

# Launch visualization
ros2 launch isaac_robot unified_visualization.launch.py viz_backend:=rosbridge

# In Foxglove Studio:
# Add 3D panel → /fused/pointcloud
# Add Image panel → /annotations/image
```

### Example 3: Multi-View Dashboard

Create a layout with:
- Top row: Raw camera feeds (`/camera_front/...`, `/camera_rear/...`)
- Middle row: Processed streams (`/processed/...`, `/fused/...`)
- Bottom row: System metrics (plots for CPU, memory, temperature)

## See Also

- [Visualization Guide](VISUALIZATION.md) - General visualization setup
- [Quick Start](QUICK_START.md) - Quick reference
- [Foxglove Documentation](https://docs.foxglove.dev/) - Foxglove Studio features
