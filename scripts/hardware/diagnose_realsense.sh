#!/bin/bash
# RealSense Camera Diagnostics Script
# Tests both RealSense cameras and verifies functionality

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo "=========================================="
echo "RealSense Camera Diagnostics"
echo "=========================================="
echo ""

# Check if RealSense SDK is installed
echo -e "${GREEN}Checking RealSense SDK installation...${NC}"
if python3 -c "import pyrealsense2" 2>/dev/null; then
    RS_VERSION=$(python3 -c "import pyrealsense2; print(pyrealsense2.__version__)" 2>/dev/null || echo "unknown")
    echo "✓ pyrealsense2 installed (version: $RS_VERSION)"
else
    echo -e "${RED}✗ pyrealsense2 not installed${NC}"
    echo "  Run: sudo ./scripts/hardware/install_realsense.sh"
    exit 1
fi

# Check USB devices
echo ""
echo -e "${GREEN}Checking USB devices...${NC}"
REALSENSE_COUNT=$(lsusb | grep -i "8086:0b07" | wc -l)
if [ "$REALSENSE_COUNT" -eq 2 ]; then
    echo "✓ Found 2 RealSense D435 cameras"
    lsusb | grep -i "8086:0b07"
elif [ "$REALSENSE_COUNT" -eq 1 ]; then
    echo -e "${YELLOW}⚠ Found only 1 RealSense camera (expected 2)${NC}"
    lsusb | grep -i "8086:0b07"
elif [ "$REALSENSE_COUNT" -eq 0 ]; then
    echo -e "${RED}✗ No RealSense cameras detected${NC}"
    exit 1
else
    echo -e "${YELLOW}⚠ Found $REALSENSE_COUNT RealSense cameras${NC}"
    lsusb | grep -i "8086:0b07"
fi

# Check USB permissions
echo ""
echo -e "${GREEN}Checking USB permissions...${NC}"
if groups | grep -q dialout; then
    echo "✓ User is in dialout group"
else
    echo -e "${YELLOW}⚠ User not in dialout group${NC}"
    echo "  Run: sudo usermod -a -G dialout $USER"
    echo "  Then log out and back in"
fi

# Test cameras with Python script
echo ""
echo -e "${GREEN}Testing camera connectivity...${NC}"

# Create temporary test script
TEST_SCRIPT=$(mktemp)
cat > "$TEST_SCRIPT" << 'PYTHON_EOF'
#!/usr/bin/env python3
import pyrealsense2 as rs
import sys

try:
    # Create context
    ctx = rs.context()
    devices = ctx.query_devices()

    if len(devices) == 0:
        print("ERROR: No RealSense devices found")
        sys.exit(1)

    print(f"Found {len(devices)} RealSense device(s):")

    for i, dev in enumerate(devices):
        print(f"\n--- Camera {i+1} ---")
        print(f"  Name: {dev.get_info(rs.camera_info.name)}")
        print(f"  Serial: {dev.get_info(rs.camera_info.serial_number)}")
        print(f"  Firmware: {dev.get_info(rs.camera_info.firmware_version)}")
        print(f"  USB Type: {dev.get_info(rs.camera_info.usb_type_descriptor)}")

        # Try to start streams
        try:
            pipeline = rs.pipeline()
            config = rs.config()
            config.enable_device(dev.get_info(rs.camera_info.serial_number))
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)

            pipeline.start(config)
            print("  ✓ Streams started successfully")

            # Get a few frames
            for _ in range(10):
                frames = pipeline.wait_for_frames(timeout_ms=5000)
                depth_frame = frames.get_depth_frame()
                color_frame = frames.get_color_frame()

                if depth_frame and color_frame:
                    print(f"  ✓ Frames received: depth={depth_frame.get_width()}x{depth_frame.get_height()}, color={color_frame.get_width()}x{color_frame.get_height()}")
                    break
            else:
                print("  ⚠ No frames received")

            pipeline.stop()

        except Exception as e:
            print(f"  ✗ Error starting streams: {e}")
            sys.exit(1)

    print("\n✓ All cameras tested successfully")
    sys.exit(0)

except Exception as e:
    print(f"ERROR: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
PYTHON_EOF

chmod +x "$TEST_SCRIPT"

# Run test
if python3 "$TEST_SCRIPT"; then
    echo -e "${GREEN}✓ Camera connectivity test passed${NC}"
else
    echo -e "${RED}✗ Camera connectivity test failed${NC}"
    rm -f "$TEST_SCRIPT"
    exit 1
fi

rm -f "$TEST_SCRIPT"

# Check ROS 2 wrapper if ROS 2 is installed
echo ""
if [ -f "/opt/ros/humble/setup.bash" ]; then
    echo -e "${GREEN}Checking ROS 2 RealSense wrapper...${NC}"
    ROS2_WS="${ROS2_WS:-$HOME/ros2_ws}"
    if [ -d "$ROS2_WS/install/realsense2_camera" ]; then
        echo "✓ ROS 2 RealSense wrapper is built"
    else
        echo -e "${YELLOW}⚠ ROS 2 RealSense wrapper not built${NC}"
        echo "  Build with:"
        echo "    cd $ROS2_WS"
        echo "    source /opt/ros/humble/setup.bash"
        echo "    colcon build --packages-select realsense2_camera"
    fi
fi

echo ""
echo "=========================================="
echo "Diagnostics complete!"
echo "=========================================="
