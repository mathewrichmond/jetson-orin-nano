# Camera Data Not Showing in Foxglove Studio

If camera images aren't appearing in Foxglove Studio, follow these diagnostic steps:

## Step 1: Check if Camera Node is Running

```bash
# Check if realsense camera node is running
ros2 node list | grep realsense

# Should show: /realsense_camera_node
```

If not running, start the graph:
```bash
./scripts/system/manage_graph.sh start robot
```

## Step 2: Check if Camera Topics Exist

```bash
# List all camera topics
ros2 topic list | grep camera

# Should see topics like:
# /camera_front/color/image_raw
# /camera_front/color/camera_info
# /camera_front/depth/image_rect_raw
# /camera_front/depth/camera_info
# /camera_rear/color/image_raw
# /camera_rear/color/camera_info
# /camera_rear/depth/image_rect_raw
# /camera_rear/depth/camera_info
```

If topics don't exist, cameras may not be detected.

## Step 3: Check Camera Status

```bash
# Check camera status messages
ros2 topic echo /realsense/status

# Should see status messages indicating camera initialization
```

## Step 4: Verify Cameras are Detected

```bash
# Check if RealSense cameras are detected by the system
rs-enumerate-devices

# Or check USB devices
lsusb | grep -i intel
```

## Step 5: Check if Topics are Publishing Data

```bash
# Check topic frequency (should show ~15 Hz after our frame rate reduction)
ros2 topic hz /camera_front/color/image_raw

# Check topic type
ros2 topic type /camera_front/color/image_raw
# Should show: sensor_msgs/msg/Image

# Try echoing one frame (will show image data)
ros2 topic echo /camera_front/color/image_raw --once
```

## Step 6: Check Foxglove Bridge Connection

In Foxglove Studio:
1. Check connection indicator (should be green)
2. Check if topics appear in the topic list
3. Look for any error messages

## Step 7: Verify Layout Topic Names Match

The layout expects these exact topic names:
- `/camera_front/color/image_raw`
- `/camera_front/color/camera_info`
- `/camera_front/depth/image_rect_raw`
- `/camera_front/depth/camera_info`
- `/camera_rear/color/image_raw`
- `/camera_rear/color/camera_info`
- `/camera_rear/depth/image_rect_raw`
- `/camera_rear/depth/camera_info`

If your topics have different names (e.g., `/hardware/camera_front/...`), you'll need to update the layout.

## Common Issues

### Issue: Topics Exist but No Data

**Symptoms**: Topics listed but `ros2 topic hz` shows 0 Hz

**Solutions**:
1. Check camera power/USB connection
2. Check camera permissions: `ls -l /dev/video*`
3. Restart camera node: `./scripts/system/manage_graph.sh restart robot`

### Issue: Cameras Not Detected

**Symptoms**: No camera topics, `rs-enumerate-devices` shows no devices

**Solutions**:
1. Check USB connections
2. Check USB power (may need powered hub)
3. Check USB permissions: `groups` (should include `video` group)
4. Try unplugging and replugging cameras

### Issue: Wrong Topic Names

**Symptoms**: Topics exist but have different names (e.g., `/hardware/camera_front/...`)

**Solutions**:
1. Check actual topic names: `ros2 topic list | grep camera`
2. Update layout file with correct topic names
3. Or update camera node namespace configuration

### Issue: Bridge Not Receiving Topics

**Symptoms**: Topics exist and publishing, but Foxglove Studio doesn't see them

**Solutions**:
1. Restart bridge: `./scripts/system/manage_graph.sh restart robot`
2. Check bridge logs: `./scripts/system/manage_graph.sh logs | grep foxglove`
3. Verify connection: Check connection indicator in Foxglove Studio

## Quick Diagnostic Command

Run this to check everything at once:

```bash
echo "=== Camera Nodes ==="
ros2 node list | grep -i camera || echo "No camera nodes found"

echo ""
echo "=== Camera Topics ==="
ros2 topic list | grep camera || echo "No camera topics found"

echo ""
echo "=== Camera Status ==="
timeout 2 ros2 topic echo /realsense/status --once 2>/dev/null || echo "No status messages"

echo ""
echo "=== Topic Publishing ==="
ros2 topic hz /camera_front/color/image_raw --window 5 2>/dev/null || echo "Topic not publishing"
```
