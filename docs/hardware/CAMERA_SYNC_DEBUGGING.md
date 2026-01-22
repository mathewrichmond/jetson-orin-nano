# Camera Sync Debugging Guide

This guide helps diagnose and fix camera synchronization issues, particularly firmware-related problems.

## Quick Diagnostic

Run the camera sync diagnostic script:

```bash
./scripts/hardware/check_camera_sync.sh
```

This will check:
- Firmware versions
- Sync mode configuration
- Hardware sync status
- Frame timestamp synchronization

## Common Issues

### Firmware Mismatch

**Symptom:** Sync warnings with high delta values (>5ms), inconsistent frame timing

**Diagnosis:**
```bash
rs-enumerate-devices | grep -E "(Serial|Firmware|Recommended)"
```

**Solution:** Update both cameras to the same firmware version (recommended: 5.17.0.10)

### Sync Mode Not Set

**Symptom:** Cameras not synchronized, no sync status messages

**Diagnosis:**
```bash
python3 -c "
import pyrealsense2 as rs
ctx = rs.context()
devices = ctx.query_devices()
for dev in devices:
    sensor = dev.first_depth_sensor()
    if sensor.supports(rs.option.inter_cam_sync_mode):
        mode = int(sensor.get_option(rs.option.inter_cam_sync_mode))
        print(f'Serial {dev.get_info(rs.camera_info.serial_number)}: Mode {mode}')
"
```

**Solution:** Ensure `enable_inter_cam_sync: true` in config and restart camera node

### Hardware Connection Issues

**Symptom:** Sync configured but not working, frame timeouts

**Check:**
1. Sync cable connected: Master Pin 5 → Slave Pin 5
2. Ground connected: Both cameras Pin 9 → common ground
3. USB connections stable (check `lsusb`)
4. USB power adequate (both cameras on USB 3.0 if possible)

## Firmware Update Procedure

**WARNING:** Firmware updates can brick cameras if interrupted. Ensure stable power and don't disconnect cameras during update.

### Step 1: Stop Camera Node

```bash
systemctl --user stop isaac-robot.service
```

### Step 2: Check Current Firmware

```bash
rs-enumerate-devices | grep -E "(Serial|Firmware|Recommended)"
```

### Step 3: List Available Updates

```bash
rs-fw-update -l
```

### Step 4: Update Firmware

Update cameras one at a time:

```bash
# Update camera 1 (by serial number)
rs-fw-update -s 141722072975 -f <firmware_file.bin>

# Update camera 2 (by serial number)
rs-fw-update -s 141722074840 -f <firmware_file.bin>
```

Or update to recommended version automatically:

```bash
# This will update to the recommended version
rs-fw-update -r
```

**Note:** The `-r` flag updates all cameras to their recommended firmware version.

### Step 5: Verify Firmware

```bash
rs-enumerate-devices | grep -E "(Serial|Firmware)"
```

Both cameras should show the same firmware version.

### Step 6: Restart Camera Node

```bash
systemctl --user start isaac-robot.service
```

### Step 7: Verify Sync

```bash
# Check sync status
ros2 topic echo /hardware/realsense/status

# Run diagnostic
./scripts/hardware/check_camera_sync.sh
```

## Current Status

Based on diagnostic run:

- **Camera 1 (141722072975):** FW 5.14.0 → Recommended 5.17.0.10
- **Camera 2 (141722074840):** FW 5.13.0.50 → Recommended 5.17.0.10
- **Sync Modes:** Correctly configured (Master/Slave)
- **Issue:** Firmware mismatch causing sync drift

## Expected Sync Performance

With proper hardware sync:
- Average delta: < 1 ms
- Max delta: < 5 ms
- 95%+ of frames within tolerance

## Troubleshooting Steps

1. **Check firmware versions** - Must match
2. **Verify sync modes** - Master/Slave correctly set
3. **Check hardware connections** - Sync cables and ground
4. **Test frame capture** - Both cameras producing frames
5. **Monitor sync status** - Check `/hardware/realsense/status` topic
6. **Update firmware if needed** - Use `rs-fw-update`

## Additional Resources

- [RealSense Firmware Releases](https://dev.realsenseai.com/docs/firmware-releases-d400)
- [Hardware Sync Documentation](CAMERA_FRAME_SYNC.md)
- [RealSense SDK Documentation](https://github.com/IntelRealSense/librealsense)
