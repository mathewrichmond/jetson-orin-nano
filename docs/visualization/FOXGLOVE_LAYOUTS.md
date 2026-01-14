# Foxglove Studio Layouts

This document describes the available Foxglove Studio layouts for visualizing robot data.

## Available Layouts

### 1. `foxglove_bridged.json` - Optimized for Remote/Bridged Connection

**Purpose:** Low-bandwidth layout for remote visualization via Foxglove Bridge.

**Features:**
- ✅ Camera images (front and rear)
- ✅ System stats (CPU, GPU, memory)
- ✅ Filtered/synced IMU data only
- ✅ Synced chassis data (battery, status)
- ❌ No 3D pointclouds (reduces bandwidth)
- ❌ No raw sensor data (use synced topics only)

**Topics Used:**
- `/hardware/camera_front/color/image_raw`
- `/hardware/camera_rear/color/image_raw`
- `/sensor_sync/imu/filtered` (all IMU data)
- `/sensor_sync/chassis/battery`
- `/sensor_sync/chassis/status`
- `/system/.*` (system stats)

**When to Use:**
- Remote visualization via Foxglove Bridge
- Limited bandwidth connections
- Production monitoring
- When you only need synced sensor data

### 2. `foxglove_raw.json` - Full Data for Direct Connection

**Purpose:** Complete layout with all topics for direct ROS 2 connection.

**Features:**
- ✅ Camera images (front and rear)
- ✅ 3D pointclouds (nvblox downsampled)
- ✅ System stats
- ✅ Raw IMU data (`/phat/imu`)
- ✅ Filtered/synced IMU data (`/sensor_sync/imu/filtered`)
- ✅ Raw chassis data (`/irobot/battery`, `/irobot/status`)
- ✅ Synced chassis data (`/sensor_sync/chassis/.*`)
- ✅ Audio data

**Topics Used:**
- All camera topics (color, depth, camera_info)
- `/nvblox/.*/points_downsampled` (3D pointclouds)
- `/phat/imu` (raw IMU)
- `/sensor_sync/imu/filtered` (filtered IMU)
- `/irobot/battery` (raw battery)
- `/irobot/status` (raw status)
- `/sensor_sync/chassis/.*` (synced chassis)
- `/system/.*` (system stats)
- `/microphone/audio`

**When to Use:**
- Direct ROS 2 connection (no bridge)
- Local development/debugging
- When you need raw sensor data
- When you need 3D visualization
- High-bandwidth connections

### 3. `foxglove_all_sensors.json` - Legacy Layout

**Purpose:** Original layout (being phased out in favor of bridged/raw).

**Status:** Updated to use synced topics, but consider using `foxglove_bridged.json` or `foxglove_raw.json` instead.

## Loading Layouts in Foxglove Studio

### Method 1: Import Layout File

1. Open Foxglove Studio
2. Go to **Layouts** → **Import layout**
3. Select the JSON file:
   - `config/visualization/foxglove_bridged.json` (for bridge)
   - `config/visualization/foxglove_raw.json` (for direct connection)

### Method 2: Copy Layout Content

1. Open the layout JSON file
2. Copy the entire contents
3. In Foxglove Studio: **Layouts** → **Import layout** → Paste JSON

## Bridge Configuration

The bridge is configured to only forward synced topics to reduce bandwidth:

```yaml
foxglove_bridge:
  parameters:
    topic_whitelist:
      - "/sensor_sync/.*"  # All synchronized sensor data
      - "/phat/status"
      - "/system/.*"
      - "/hardware/.*/color/image_raw"
      - "/hardware/.*/color/camera_info"
      - "/microphone/audio"
```

**Note:** Raw high-frequency topics (`/phat/imu`, `/irobot/battery`, `/irobot/status`) are **not** bridged. Use synced topics instead.

## Recommendations

1. **For Remote Monitoring:** Use `foxglove_bridged.json` with Foxglove Bridge
2. **For Local Development:** Use `foxglove_raw.json` with direct ROS 2 connection
3. **For Debugging:** Use `foxglove_raw.json` to compare raw vs synced data

## Troubleshooting

### No IMU Data in Bridged Layout

- **Problem:** IMU plot shows no data
- **Solution:** Ensure you're using `/sensor_sync/imu/filtered` topics (not `/phat/imu`)
- **Check:** Bridge whitelist includes `/sensor_sync/.*`

### Camera Images Slow

- **Problem:** Camera images update slowly
- **Solution:**
  - Use `foxglove_bridged.json` (optimized for bridge)
  - Check camera FPS is set to 15 Hz
  - Verify bridge is not dropping connections

### Missing 3D Visualization

- **Problem:** No 3D pointclouds visible
- **Solution:**
  - Use `foxglove_raw.json` (includes 3D panels)
  - Ensure nvblox processor is running
  - Check `/nvblox/.*/points_downsampled` topics are publishing
