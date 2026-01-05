# Bench Test: Visualize Cameras from MacBook

Quick guide to visualize RealSense camera data from your MacBook.

## Prerequisites

### On Jetson (Target)
- RealSense cameras connected
- ROS 2 Humble installed
- Network connection (WiFi or Ethernet)

### On MacBook
- Foxglove Studio installed (or use web browser)
- Same network as Jetson (or network access to Jetson)

## Quick Start

### Step 1: On Jetson - Run Bench Test Script

```bash
cd ~/src/jetson-orin-nano
./scripts/visualization/bench_test_cameras.sh
```

This script will:
- Check/install rosbridge
- Show connection information
- Launch cameras + rosbridge

**Note**: Keep this terminal open. Press Ctrl+C to stop.

### Step 2: On MacBook - Connect with Foxglove Studio

#### Option A: Foxglove Studio (Recommended)

1. **Download Foxglove Studio**:
   - Visit [foxglove.dev/download](https://foxglove.dev/download)
   - Download macOS version
   - Install and open

2. **Connect**:
   - Click "Open connection"
   - Select "ROS 2" connection type
   - **Host**: Use IP address shown in Jetson terminal (e.g., `192.168.1.100`)
     - Or use `isaac.local` if mDNS is working
   - **Port**: `9090`
   - Click "Open"

3. **Visualize Camera Topics**:
   - Click "+" to add panels
   - Add **Image** panel
   - Select topic: `/camera_front/color/image_raw`
   - Add another Image panel for depth: `/camera_front/depth/image_rect_raw`
   - Repeat for rear camera if available

#### Option B: Web Browser

1. **Open browser**:
   - Go to [studio.foxglove.dev](https://studio.foxglove.dev)

2. **Connect**:
   - Click "Open connection"
   - Select "ROS 2"
   - Enter Jetson IP and port `9090`
   - Connect

3. **Add camera panels** as above

## Manual Setup (Alternative)

If you prefer to launch components separately:

### On Jetson

**Terminal 1 - Cameras**:
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch realsense_camera realsense_camera.launch.py
```

**Terminal 2 - Rosbridge**:
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch isaac_robot rosbridge.launch.py rosbridge_address:=0.0.0.0
```

### On MacBook

Same as above - connect Foxglove Studio or web browser to `ws://<jetson-ip>:9090`

## Troubleshooting

### Can't Connect from MacBook

1. **Check Jetson IP address**:
   ```bash
   # On Jetson
   ip addr show
   # or
   hostname -I
   ```

2. **Test connectivity**:
   ```bash
   # On MacBook
   ping <jetson-ip>
   # or
   ping isaac.local
   ```

3. **Check rosbridge is running**:
   ```bash
   # On Jetson
   ros2 node list | grep rosbridge
   ```

4. **Check port is open**:
   ```bash
   # On Jetson
   netstat -tlnp | grep 9090
   # Should show: 0.0.0.0:9090
   ```

5. **Check firewall**:
   ```bash
   # On Jetson
   sudo ufw status
   # If active, allow port 9090:
   sudo ufw allow 9090/tcp
   ```

### No Camera Topics Visible

1. **Check cameras are publishing**:
   ```bash
   # On Jetson
   ros2 topic list | grep camera
   ros2 topic hz /camera_front/color/image_raw
   ```

2. **Check camera node is running**:
   ```bash
   # On Jetson
   ros2 node list | grep realsense
   ```

3. **Check camera status**:
   ```bash
   # On Jetson
   ros2 topic echo /realsense/status
   ```

### Connection Drops

1. **Check network stability**: `ping <jetson-ip>` continuously
2. **Check Jetson resources**: High CPU/memory can cause issues
3. **Reduce image resolution** in camera config if needed

## Visualizing Other Topics

Once connected, you can visualize **any ROS 2 topics**:

- **System metrics**: `/system/temperature/cpu`, `/system/cpu/usage`
- **Processed streams**: `/processed/video`, `/fused/stereo/image`
- **Pointclouds**: `/camera_front/points`, `/fused/pointcloud`
- **Custom topics**: Any topic your nodes publish

Just add panels in Foxglove Studio and select the topic from the dropdown.

## Next Steps

- Create custom layouts for your workflow
- Visualize processed/fused data streams
- Monitor system metrics alongside camera feeds
- Set up multiple visualization clients

See [Visualizing Topics](VISUALIZING_TOPICS.md) for more examples.
