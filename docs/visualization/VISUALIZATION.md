# Visualization Guide

This guide covers visualization options for the Isaac robot system, supporting cross-platform development on Ubuntu, Mac, and Windows.

## Overview

The visualization system is **topic-agnostic** - it can visualize **any ROS 2 topics**, including:
- Raw sensor data (cameras, IMU, etc.)
- Processed/fused video streams
- Pointclouds from any source
- Custom sensor data
- System metrics
- Control commands and status

The system provides generic visualization infrastructure:

1. **Foxglove Studio** (Recommended) - Cross-platform desktop app
2. **Web-based visualization** - Via rosbridge (works in any browser)
3. **RViz2** - Native Linux visualization (Ubuntu only)

**Key Principle**: Visualization is **separate** from data sources. Launch your nodes separately, then connect visualization to visualize any topics you want.

## Quick Start

### Option 1: Foxglove Studio (Cross-Platform)

**Best for**: Development on Mac, Windows, or Ubuntu

1. **Install Foxglove Studio**:
   - Download from [foxglove.dev/download](https://foxglove.dev/download)
   - Available for Ubuntu, Mac, and Windows

2. **On Robot**: Launch rosbridge (generic, works with any ROS 2 system):
   ```bash
   ros2 launch isaac_robot rosbridge.launch.py
   ```

   Or with unified visualization (includes rosbridge or RViz2):
   ```bash
   ros2 launch isaac_robot unified_visualization.launch.py viz_backend:=rosbridge
   ```

3. **In Foxglove Studio**:
   - Click "Open connection"
   - Select **"WebSocket"** connection type (NOT "ROS 2" - that's for Foxglove Bridge)
   - Enter: `ws://192.168.0.155:9090` (or `ws://isaac.local:9090` if mDNS configured)
   - Click "Open"

   **Note**: If you see instructions for Foxglove Bridge, you selected "ROS 2" by mistake. Use "WebSocket" instead.

4. **Load Layout** (optional):
   - File → Import Layout
   - Select `src/hardware_drivers/realsense_camera/config/foxglove_layout.json` (for cameras)
   - Or create your own custom layout

### Option 2: Web-Based Visualization (Docker)

**Best for**: Docker deployments, remote access via browser

1. **Start Docker visualization service**:
   ```bash
   docker-compose up isaac-viz
   ```

2. **Access via browser**:
   - Open [studio.foxglove.dev](https://studio.foxglove.dev)
   - Or use custom web UI connecting to `ws://<robot-ip>:9090`

3. **Connect**:
   - Select "ROS 2" connection
   - Enter robot IP and port `9090`

### Option 3: RViz2 (Linux Native)

**Best for**: Native Linux development

```bash
ros2 launch isaac_robot unified_visualization.launch.py viz_backend:=rviz2 rviz_config:=/path/to/config.rviz
```

## Detailed Setup

### Prerequisites

#### Install rosbridge_suite

```bash
# On Ubuntu/Debian
sudo apt install ros-humble-rosbridge-suite

# Or build from source
cd ~/ros2_ws/src
git clone https://github.com/RobotWebTools/rosbridge_suite.git -b humble
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select rosbridge_suite
source install/setup.bash
```

### Launch Options

#### Generic Visualization (System-Wide)

These launch files work with **any ROS 2 topics**:

**Rosbridge only** (web-based):
```bash
ros2 launch isaac_robot rosbridge.launch.py
```

**Unified visualization** (rosbridge or RViz2):
```bash
# Web-based (rosbridge)
ros2 launch isaac_robot unified_visualization.launch.py viz_backend:=rosbridge

# RViz2 (Linux only)
ros2 launch isaac_robot unified_visualization.launch.py viz_backend:=rviz2

# No visualization
ros2 launch isaac_robot unified_visualization.launch.py viz_backend:=none
```

**Custom rosbridge port**:
```bash
ros2 launch isaac_robot rosbridge.launch.py rosbridge_port:=9091
```

### Visualizing Your Topics

Once visualization is running, you can visualize **any topics**:

1. **List available topics**:
   ```bash
   ros2 topic list
   ```

2. **In Foxglove Studio or web UI**:
   - Add panels (Image, PointCloud, Plot, etc.)
   - Select any topic from the dropdown
   - Configure as needed

3. **Examples**:
   - Raw camera: `/camera_front/color/image_raw`
   - Processed video: `/processed/video`, `/fused/stereo/image`
   - Pointclouds: `/camera_front/points`, `/fused/pointcloud`
   - System metrics: `/system/temperature/cpu`, `/system/cpu/usage`

See [Visualizing Topics](VISUALIZING_TOPICS.md) for detailed examples and best practices.

### Docker Deployment

#### Using Docker Compose

```bash
# Start visualization service
docker-compose up -d isaac-viz

# View logs
docker-compose logs -f isaac-viz

# Stop
docker-compose stop isaac-viz
```

#### Manual Docker Run

```bash
docker run -it --rm \
  --network host \
  -e ROS_DOMAIN_ID=0 \
  -e ROSBRIDGE_PORT=9090 \
  -v $(pwd):/workspace \
  -v ~/ros2_ws:/home/nano/ros2_ws \
  isaac-ros:latest \
  bash -c "source /opt/ros/humble/setup.bash && \
           source ~/ros2_ws/install/setup.bash && \
           ros2 launch isaac_robot rosbridge.launch.py"
```

## Foxglove Studio Configuration

### Connection Settings

- **Connection Type**: ROS 2
- **Host**: Robot IP address (e.g., `192.168.1.100` or `isaac.local`)
- **Port**: `9090` (default rosbridge port)
- **ROS_DOMAIN_ID**: Must match robot's ROS_DOMAIN_ID (default: 0)

### Layout Configuration

A pre-configured layout is available at:
```
src/hardware_drivers/realsense_camera/config/foxglove_layout.json
```

This layout includes:
- Camera front/rear color images
- Camera front/rear depth images
- System temperature plots
- CPU/Memory usage plots
- Camera status messages

### Custom Layouts

Create custom layouts in Foxglove Studio:
1. Arrange panels as desired
2. File → Export Layout
3. Save for reuse

## Web-Based Visualization

### Using Foxglove Web

1. Open [studio.foxglove.dev](https://studio.foxglove.dev) in browser
2. Connect to robot's rosbridge server
3. No installation required

### Custom Web UI

You can build custom web UIs using:
- **roslibjs** - JavaScript library for ROS
- **rosbridge** - WebSocket bridge to ROS

Example connection:
```javascript
const ros = new ROSLIB.Ros({
  url: 'ws://<robot-ip>:9090'
});
```

## Remote Visualization

### Same Network (ROS_DOMAIN_ID)

1. **On Robot**: Set ROS_DOMAIN_ID and launch visualization
   ```bash
   export ROS_DOMAIN_ID=0
   ros2 launch isaac_robot rosbridge.launch.py
   ```

2. **On Remote Machine**: Set same ROS_DOMAIN_ID
   ```bash
   export ROS_DOMAIN_ID=0
   # Then connect via Foxglove Studio or web UI
   ```

### Different Networks (rosbridge)

1. **On Robot**: Launch with rosbridge bound to all interfaces
   ```bash
   ros2 launch isaac_robot rosbridge.launch.py \
     rosbridge_address:=0.0.0.0 rosbridge_port:=9090
   ```

2. **On Remote Machine**: Connect via robot's public IP
   - Use Foxglove Studio or web UI
   - Connect to `ws://<robot-public-ip>:9090`

**Note**: Ensure firewall allows port 9090 if connecting across networks.

## Troubleshooting

### Rosbridge Not Connecting

1. **Check rosbridge is running**:
   ```bash
   ros2 node list | grep rosbridge
   ```

2. **Check port is open**:
   ```bash
   netstat -tlnp | grep 9090
   ```

3. **Check firewall**:
   ```bash
   sudo ufw status
   sudo ufw allow 9090/tcp  # If needed
   ```

### Foxglove Studio Connection Issues

1. **Verify ROS_DOMAIN_ID matches** on both machines
2. **Check network connectivity**: `ping <robot-ip>`
3. **Verify rosbridge port**: Default is 9090
4. **Check robot's rosbridge logs** for errors

### Docker Issues

1. **Port conflicts**: Change `ROSBRIDGE_PORT` in docker-compose.yml
2. **Network mode**: Ensure `network_mode: host` or ports are exposed
3. **ROS_DOMAIN_ID**: Must match between containers and host

## Advanced Configuration

### Multiple Visualization Clients

rosbridge supports multiple simultaneous connections. You can have:
- Multiple Foxglove Studio instances
- Web UI + Foxglove Studio
- Any combination of clients

### Performance Tuning

For high-bandwidth topics (images), reduce resolution in your data nodes:

```bash
# Example: Launch cameras with lower resolution
ros2 launch realsense_camera realsense_camera.launch.py \
  config_file:=/path/to/low_res_config.yaml

# Launch visualization separately
ros2 launch isaac_robot rosbridge.launch.py
```

### Security Considerations

For production deployments:

1. **Use authentication**: Configure rosbridge with authentication
2. **Restrict access**: Use firewall rules to limit access
3. **Use HTTPS/WSS**: For web-based access, use secure WebSocket
4. **VPN**: Consider VPN for remote access

## References

- [Foxglove Studio Documentation](https://docs.foxglove.dev/)
- [rosbridge_suite GitHub](https://github.com/RobotWebTools/rosbridge_suite)
- [ROS 2 rosbridge Documentation](https://github.com/RobotWebTools/rosbridge_suite/blob/ros2/README.md)
