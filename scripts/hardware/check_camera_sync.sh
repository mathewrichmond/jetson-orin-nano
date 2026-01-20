#!/bin/bash
# Camera Sync Hardware/Firmware Diagnostic Script
# Checks firmware versions, sync modes, and hardware sync status

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo "=========================================="
echo "Camera Sync Hardware/Firmware Diagnostics"
echo "=========================================="
echo ""

# Check if RealSense SDK is installed
echo -e "${GREEN}Checking RealSense SDK...${NC}"
if ! python3 -c "import pyrealsense2" 2>/dev/null; then
    echo -e "${RED}✗ pyrealsense2 not installed${NC}"
    exit 1
fi
RS_VERSION=$(python3 -c "import pyrealsense2; print(pyrealsense2.__version__)" 2>/dev/null || echo "unknown")
echo "✓ pyrealsense2 version: $RS_VERSION"
echo ""

# Run comprehensive hardware/firmware check
echo -e "${GREEN}Hardware/Firmware Status:${NC}"
python3 << 'PYTHON_EOF'
import pyrealsense2 as rs
import sys

try:
    ctx = rs.context()
    devices = ctx.query_devices()
    
    if len(devices) == 0:
        print("ERROR: No RealSense devices found")
        sys.exit(1)
    
    print(f"Found {len(devices)} camera(s):\n")
    
    firmware_versions = []
    sync_modes = []
    
    for i, dev in enumerate(devices):
        serial = dev.get_info(rs.camera_info.serial_number)
        fw = dev.get_info(rs.camera_info.firmware_version)
        recommended = dev.get_info(rs.camera_info.recommended_firmware_version)
        name = dev.get_info(rs.camera_info.name)
        usb_type = dev.get_info(rs.camera_info.usb_type_descriptor)
        
        print(f"Camera {i+1}:")
        print(f"  Serial: {serial}")
        print(f"  Name: {name}")
        print(f"  Current FW: {fw}")
        print(f"  Recommended FW: {recommended}")
        print(f"  USB Type: {usb_type}")
        
        firmware_versions.append(fw)
        
        # Check sync support and current mode
        sensor = dev.first_depth_sensor()
        supports_sync = sensor.supports(rs.option.inter_cam_sync_mode)
        print(f"  Supports inter_cam_sync_mode: {supports_sync}")
        
        if supports_sync:
            try:
                current_mode = int(sensor.get_option(rs.option.inter_cam_sync_mode))
                mode_names = {0: 'None', 1: 'Master', 2: 'Slave'}
                mode_name = mode_names.get(current_mode, f'Unknown({current_mode})')
                print(f"  Current sync mode: {current_mode} ({mode_name})")
                sync_modes.append(current_mode)
            except Exception as e:
                print(f"  Error reading sync mode: {e}")
                sync_modes.append(None)
        else:
            sync_modes.append(None)
            print(f"  ⚠ Camera does not support hardware sync")
        
        print()
    
    # Check for firmware mismatch
    if len(set(firmware_versions)) > 1:
        print("⚠ WARNING: Firmware version mismatch detected!")
        print("  Different firmware versions can cause sync issues.")
        print("  Recommendation: Update both cameras to the same firmware version.")
        print(f"  Recommended version: {devices[0].get_info(rs.camera_info.recommended_firmware_version)}")
        print()
    
    # Check sync mode configuration
    if len(sync_modes) >= 2:
        if sync_modes[0] == 1 and sync_modes[1] == 2:
            print("✓ Sync mode configuration correct (Master/Slave)")
        elif sync_modes[0] == 2 and sync_modes[1] == 1:
            print("✓ Sync mode configuration correct (Slave/Master)")
        elif sync_modes[0] == 0 or sync_modes[1] == 0:
            print("⚠ WARNING: One or both cameras have sync disabled")
        else:
            print("⚠ WARNING: Sync mode configuration may be incorrect")
        print()
    
    # Check if firmware is outdated
    for i, dev in enumerate(devices):
        fw = dev.get_info(rs.camera_info.firmware_version)
        recommended = dev.get_info(rs.camera_info.recommended_firmware_version)
        if fw != recommended:
            print(f"⚠ Camera {i+1} firmware is outdated:")
            print(f"  Current: {fw}")
            print(f"  Recommended: {recommended}")
            print()
    
except Exception as e:
    print(f"ERROR: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
PYTHON_EOF

echo ""
echo -e "${GREEN}Testing Frame Capture with Sync Enabled:${NC}"
echo ""

# Test frame capture and timestamp sync
python3 << 'PYTHON_EOF'
import pyrealsense2 as rs
import time
import sys

try:
    ctx = rs.context()
    devices = ctx.query_devices()
    
    if len(devices) < 2:
        print("⚠ Need 2 cameras for sync test, found:", len(devices))
        sys.exit(0)
    
    pipelines = []
    cameras_info = []
    
    # Initialize pipelines
    for i, dev in enumerate(devices):
        serial = dev.get_info(rs.camera_info.serial_number)
        sensor = dev.first_depth_sensor()
        
        # Get current sync mode
        sync_mode = 0
        if sensor.supports(rs.option.inter_cam_sync_mode):
            sync_mode = int(sensor.get_option(rs.option.inter_cam_sync_mode))
        
        cameras_info.append({
            'serial': serial,
            'sync_mode': sync_mode,
            'mode_name': ['None', 'Master', 'Slave'][sync_mode] if sync_mode < 3 else 'Unknown'
        })
        
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(serial)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 15)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
        
        try:
            profile = pipeline.start(config)
            pipelines.append(pipeline)
            print(f"✓ Started pipeline for camera {i+1} (Serial: {serial[:8]}..., Mode: {cameras_info[i]['mode_name']})")
        except Exception as e:
            print(f"✗ Failed to start pipeline for camera {i+1}: {e}")
            sys.exit(1)
    
    print("\nCollecting frames for sync analysis...")
    time.sleep(2)  # Let cameras stabilize
    
    # Collect timestamp pairs
    timestamp_pairs = []
    max_samples = 30
    
    for sample in range(max_samples):
        frames_by_camera = {}
        
        for i, pipeline in enumerate(pipelines):
            try:
                frames = pipeline.wait_for_frames(timeout_ms=2000)
                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()
                
                if color_frame:
                    # Use hardware timestamp from metadata if available
                    try:
                        ts_ms = color_frame.get_frame_metadata(rs.frame_metadata_value.sensor_timestamp)
                    except:
                        ts_ms = color_frame.get_timestamp()
                    
                    frames_by_camera[i] = {
                        'timestamp_ms': ts_ms,
                        'frame_number': color_frame.get_frame_number()
                    }
            except Exception as e:
                print(f"  Warning: Camera {i+1} frame timeout: {e}")
                break
        
        if len(frames_by_camera) == 2:
            ts1 = frames_by_camera[0]['timestamp_ms']
            ts2 = frames_by_camera[1]['timestamp_ms']
            delta_ms = abs(ts1 - ts2)
            timestamp_pairs.append({
                'ts1': ts1,
                'ts2': ts2,
                'delta_ms': delta_ms,
                'frame1': frames_by_camera[0]['frame_number'],
                'frame2': frames_by_camera[1]['frame_number']
            })
        
        time.sleep(0.1)
    
    # Stop pipelines
    for pipeline in pipelines:
        try:
            pipeline.stop()
        except:
            pass
    
    if len(timestamp_pairs) == 0:
        print("✗ No synchronized frame pairs collected")
        sys.exit(1)
    
    # Analyze sync quality
    deltas = [p['delta_ms'] for p in timestamp_pairs]
    avg_delta = sum(deltas) / len(deltas)
    max_delta = max(deltas)
    min_delta = min(deltas)
    
    print(f"\nSync Analysis (from {len(timestamp_pairs)} frame pairs):")
    print(f"  Average delta: {avg_delta:.2f} ms")
    print(f"  Min delta: {min_delta:.2f} ms")
    print(f"  Max delta: {max_delta:.2f} ms")
    
    # Check against tolerance (5ms)
    tolerance_ms = 5.0
    if avg_delta <= tolerance_ms:
        print(f"  ✓ Average sync delta within tolerance ({tolerance_ms} ms)")
    else:
        print(f"  ⚠ Average sync delta exceeds tolerance ({tolerance_ms} ms)")
    
    if max_delta <= tolerance_ms:
        print(f"  ✓ Max sync delta within tolerance ({tolerance_ms} ms)")
    else:
        print(f"  ⚠ Max sync delta exceeds tolerance ({tolerance_ms} ms)")
    
    # Count frames within tolerance
    within_tolerance = sum(1 for d in deltas if d <= tolerance_ms)
    percentage = (within_tolerance / len(deltas)) * 100
    print(f"  Frames within tolerance: {within_tolerance}/{len(deltas)} ({percentage:.1f}%)")
    
    if percentage < 80:
        print("\n⚠ WARNING: Less than 80% of frames are within sync tolerance")
        print("  This indicates hardware sync may not be working correctly.")
        print("  Possible causes:")
        print("    - Firmware mismatch")
        print("    - Sync cable connection issues")
        print("    - USB bandwidth/power issues")
        print("    - Camera initialization timing")
    
except Exception as e:
    print(f"ERROR: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
PYTHON_EOF

echo ""
echo "=========================================="
echo "Diagnostics complete!"
echo "=========================================="
echo ""
echo -e "${BLUE}Next Steps:${NC}"
echo "1. If firmware versions differ, update both cameras to the same version"
echo "2. Verify sync cable connections (Pin 5 sync, Pin 9 ground)"
echo "3. Check USB connections and power"
echo "4. Restart camera node: ./scripts/system/manage_graph.sh restart robot"
echo ""
