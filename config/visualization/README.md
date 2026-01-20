# Visualization Configurations

This directory contains Foxglove Studio layout configurations for visualizing robot sensor data.

**Foxglove Studio Version**: 2.43.0

## Available Layouts

### `sensor_bridged.json` - Remote/Bridged Connection (Low Bandwidth)

**Purpose**: Optimized for remote monitoring via Foxglove Bridge

**Topics Used**:
- `/viz/remote/camera_front/color/image_raw` - Downsampled camera (320×240 @ 10 Hz)
- `/viz/remote/imu/filtered` - Filtered IMU data
- `/viz/remote/chassis/battery` - Synced battery data
- `/viz/remote/three_d/mesh` - Decimated fused mesh (75% reduction @ 5 Hz)
- `/viz/remote/three_d/pointcloud` - Downsampled pointcloud (factor 16 @ 5 Hz)
- `/system/*` - System monitoring

**Features**:
- Front camera only (reduces bandwidth)
- Aggressively downsampled 3D data
- Essential sensors only
- Optimized for 5 MB/s bridge budget

**Use Case**: Remote monitoring, debugging over network

### `sensor_raw.json` - Direct Connection (Full Quality)

**Purpose**: Full-quality visualization for direct ROS 2 connection

**Topics Used**:
- `/hardware/camera_*/color/image_raw` - Raw camera images (640×480 @ 15 Hz)
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

### `camera_debug.json` - Camera Debugging Layout

**Purpose**: Camera debugging and monitoring with frame rate visualization

**Topics Used**:
- `/viz/remote/camera_front/color/image_raw` - Downsampled front camera (320×240 @ 10 Hz)
- `/viz/remote/camera_rear/color/image_raw` - Downsampled rear camera (320×240 @ 10 Hz)
- `/hardware/camera_*/color/camera_info` - Camera calibration info
- `/hardware/camera_*/depth/camera_info` - Depth camera calibration info
- `/hardware/camera_*/color/image_raw.header.stamp.*` - Timestamp data for frame rate plots
- `/sensor_fusion/system/hardware/camera_sync_status` - Camera synchronization status

**Features**:
- Two side-by-side camera views (front and rear)
- Camera info panels showing calibration data
- Timestamp plots for frame rate monitoring
- Sync status panel
- Optimized for debugging camera publishing, sync, and frame rates

**Use Case**: Debugging camera publishing issues, monitoring frame rates, checking sync status

## Usage

### Step 1: Connect to Data Source

#### Option A: Foxglove Bridge (Remote Connection)

1. **Set up port forwarding** (if connecting from laptop):
   ```bash
   # On your laptop
   ssh -L 8765:localhost:8765 nano@isaac.local
   ```

2. **In Foxglove Studio**:
   - Connection type: **Foxglove WebSocket**
   - Host: `localhost` (if port forwarded) or `isaac.local` (if same network)
   - Port: `8765`
   - Click **Connect**

#### Option B: Direct ROS 2 Connection (Local)

1. **Ensure ROS 2 is running** on the robot
2. **In Foxglove Studio**:
   - Connection type: **ROS 2**
   - Domain ID: `0` (or match your ROS_DOMAIN_ID)
   - Click **Connect**

### Step 2: Import Layout

1. In Foxglove Studio, go to **Layouts** → **Import**
2. Navigate to `config/visualization/` directory
3. Select the appropriate layout file:
   - **Bridge connection**: `sensor_bridged.json` or `camera_debug.json`
   - **Direct connection**: `sensor_raw.json` or `camera_debug.json`

### Step 3: Verify Topics

After importing, check that topics are available:
- Look for topics in the **Topics** panel (left sidebar)
- If topics are missing, verify:
  - Bridge is running: `ss -tlnp | grep 8765` (on robot)
  - Robot graph is running: `systemctl --user status isaac-robot.service`
  - Topics are publishing: `ros2 topic list` (on robot)

## Layout Structure (Foxglove 2.43.0)

Layouts use a nested structure with `direction`, `first`, `second`, `third`, etc.:

```json
{
  "layout": {
    "direction": "row",  // or "column"
    "first": "Panel!name",  // Panel ID string
    "second": {  // Or nested layout object
      "direction": "column",
      "first": "Panel!name1",
      "second": "Panel!name2"
    }
  }
}
```

**Important Rules**:
- `"row"` direction splits horizontally (left/right)
- `"column"` direction splits vertically (top/bottom)
- Panel IDs must match entries in `configById`
- Panel IDs use format: `"Type!name"` (e.g., `"Image!front"`, `"Plot!stats"`)
- Nested layouts must have `"direction"` property
- All layout nodes must have `"first"` property (required)
- `"second"` is optional but recommended for `"row"` layouts

## Troubleshooting

### Layout Import Errors

**Error: "Cannot read properties of undefined (reading 'first')"**

This means the layout structure is invalid. Common causes:
1. Missing `"first"` property in a layout node
2. Invalid panel ID (not in `configById`)
3. Incorrect nesting structure

**Fix**: 
- Validate JSON: `python3 -m json.tool config/visualization/your_layout.json`
- Compare with working layouts (`sensor_bridged.json`, `sensor_raw.json`)
- Ensure all layout nodes have `"direction"` and `"first"` properties

### Topics Not Appearing

**Symptoms**: Layout loads but panels show "No data" or topics missing

**Check**:
1. **Bridge connection**:
   ```bash
   # On robot
   ss -tlnp | grep 8765  # Should show bridge listening
   ros2 topic list | grep "/viz/remote"  # Should show viz topics
   ```

2. **Direct connection**:
   ```bash
   # On robot
   ros2 topic list | grep "/hardware"  # Should show hardware topics
   ```

3. **Robot graph running**:
   ```bash
   systemctl --user status isaac-robot.service
   ```

**Fix**:
- Restart robot graph: `./scripts/system/manage_graph.sh restart robot`
- Check bridge logs: `./scripts/system/manage_graph.sh container-logs foxglove_bridge`
- Verify topics are in graph config: `config/robot/robot_graph.yaml`

### Port Forwarding Issues

**Symptoms**: Can't connect from laptop, "Connection refused"

**Check**:
```bash
# On laptop
netstat -an | grep 8765  # Should show LISTEN on localhost:8765
```

**Fix**:
- Ensure SSH tunnel is active: `ssh -L 8765:localhost:8765 nano@isaac.local`
- Keep SSH session open while using Foxglove
- Verify bridge is running on robot: `ss -tlnp | grep 8765`

### Layout Format Issues

**Symptoms**: Layout imports but panels are empty or misaligned

**Check**:
- Panel IDs match `configById` entries exactly
- Topic names are correct (check with `ros2 topic list`)
- QoS settings match (BEST_EFFORT vs RELIABLE)

**Fix**:
- Export layout from Foxglove Studio after manual adjustments
- Compare topic names with actual published topics
- Update layout JSON with correct topic paths

## Creating Custom Layouts

1. **Start with existing layout**: Copy `sensor_bridged.json` or `sensor_raw.json`
2. **Modify in Foxglove Studio**:
   - Import the layout
   - Add/remove panels via UI
   - Configure panel settings
   - Export: **Layouts** → **Export**
3. **Save to repository**: Copy exported JSON to `config/visualization/`
4. **Validate**: `python3 -m json.tool config/visualization/your_layout.json`

## Additional Resources

- **Detailed topic information**: `docs/visualization/FOXGLOVE_LAYOUTS.md`
- **Bridge troubleshooting**: `docs/visualization/FOXGLOVE_BRIDGE_TROUBLESHOOTING.md`
- **Visualization guide**: `docs/visualization/VISUALIZATION.md`
