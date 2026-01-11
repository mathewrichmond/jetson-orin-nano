# Audio and 3D Visualization Notes

## Audio/Sound Visualization

**Current Status**: The USB microphone node (`usb_microphone`) only publishes **status messages**, not raw audio data.

**What's Available**:
- `/microphone/status` - Status messages (text) showing microphone state
- Raw audio data is **not currently published**

**To Enable Audio Visualization**:

The microphone node would need to be modified to publish audio data. Options:

1. **Publish audio as sensor_msgs/PointCloud2** (workaround)
2. **Use foxglove.RawAudio message type** (requires custom message definition)
3. **Publish audio as std_msgs/UInt8MultiArray** (simple but not ideal)

**Current Workaround**:
- Microphone status is visible in the `RawMessages!microphone_status` panel
- Status shows: "capturing", "idle", "error" states
- To see actual audio waveforms, the node needs to publish audio data

## 3D Pointcloud Visualization

**Current Configuration**:
- Pointclouds are enabled in `config/robot/robot_graph.yaml` (`enable_pointcloud: true`)
- Topics should be: `/hardware/camera_front/points` and `/hardware/camera_rear/points`

**If 3D Panels Are Empty**:

1. **Verify pointclouds are enabled**:
   ```bash
   # Check graph config
   grep enable_pointcloud config/robot/robot_graph.yaml
   ```

2. **Check if topics exist**:
   ```bash
   ros2 topic list | grep points
   # Should show:
   # /hardware/camera_front/points
   # /hardware/camera_rear/points
   ```

3. **Check if topics are publishing**:
   ```bash
   ros2 topic hz /hardware/camera_front/points
   # Should show frequency (e.g., 15 Hz)
   ```

4. **Check topic type**:
   ```bash
   ros2 topic type /hardware/camera_front/points
   # Should show: sensor_msgs/msg/PointCloud2
   ```

5. **Verify transforms**:
   ```bash
   ros2 run tf2_ros tf2_echo camera_front_depth_optical_frame base_link
   # 3D panels need transform messages to position pointclouds
   ```

**Common Issues**:
- Pointclouds disabled in config (set `enable_pointcloud: true`)
- No transform messages (tf) - 3D panels need transforms to position data
- Topics not publishing (check camera node logs)
- Wrong topic names in layout (should include `/hardware/` namespace)

**Restart After Changes**:
```bash
./scripts/system/manage_graph.sh restart robot
```
