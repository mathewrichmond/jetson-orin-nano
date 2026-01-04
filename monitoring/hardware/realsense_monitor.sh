#!/bin/bash
# RealSense Camera Health Monitor
# Checks RealSense camera health and publishes status

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo "=========================================="
echo "RealSense Camera Health Monitor"
echo "=========================================="
echo ""

# Check if cameras are detected
echo -e "${GREEN}Checking USB devices...${NC}"
REALSENSE_COUNT=$(lsusb | grep -i "8086:0b07" | wc -l)
if [ "$REALSENSE_COUNT" -eq 2 ]; then
    echo "✓ Found 2 RealSense D435 cameras"
elif [ "$REALSENSE_COUNT" -eq 1 ]; then
    echo -e "${YELLOW}⚠ Found only 1 RealSense camera (expected 2)${NC}"
elif [ "$REALSENSE_COUNT" -eq 0 ]; then
    echo -e "${RED}✗ No RealSense cameras detected${NC}"
    exit 1
else
    echo -e "${YELLOW}⚠ Found $REALSENSE_COUNT RealSense cameras${NC}"
fi

# Check if pyrealsense2 is available
echo ""
echo -e "${GREEN}Checking RealSense SDK...${NC}"
if python3 -c "import pyrealsense2" 2>/dev/null; then
    echo "✓ pyrealsense2 available"
else
    echo -e "${RED}✗ pyrealsense2 not available${NC}"
    exit 1
fi

# Check if ROS 2 is running and cameras are publishing
echo ""
echo -e "${GREEN}Checking ROS 2 topics...${NC}"
if command -v ros2 &> /dev/null; then
    source /opt/ros/humble/setup.bash 2>/dev/null || true

    # Check for camera topics
    CAMERA_TOPICS=$(ros2 topic list 2>/dev/null | grep -E "(camera_front|camera_rear)" || true)

    if [ -z "$CAMERA_TOPICS" ]; then
        echo -e "${YELLOW}⚠ No camera topics found (cameras may not be running)${NC}"
    else
        echo "✓ Camera topics found:"
        echo "$CAMERA_TOPICS" | head -5 | sed 's/^/  /'

        # Check if topics are publishing
        echo ""
        echo -e "${GREEN}Checking topic activity...${NC}"
        for topic in /camera_front/color/image_raw /camera_rear/color/image_raw; do
            if ros2 topic info "$topic" &>/dev/null; then
                PUB_COUNT=$(ros2 topic info "$topic" 2>/dev/null | grep "Publisher count:" | awk '{print $3}' || echo "0")
                if [ "$PUB_COUNT" -gt 0 ]; then
                    echo "✓ $topic is publishing"
                else
                    echo -e "${YELLOW}⚠ $topic has no publishers${NC}"
                fi
            fi
        done
    fi
else
    echo -e "${YELLOW}⚠ ROS 2 not available${NC}"
fi

# Test camera connectivity with Python
echo ""
echo -e "${GREEN}Testing camera connectivity...${NC}"

TEST_SCRIPT=$(mktemp)
cat > "$TEST_SCRIPT" << 'PYTHON_EOF'
import pyrealsense2 as rs
import sys

try:
    ctx = rs.context()
    devices = ctx.query_devices()

    if len(devices) == 0:
        print("ERROR: No RealSense devices found")
        sys.exit(1)

    print(f"Found {len(devices)} RealSense device(s)")
    all_ok = True

    for i, dev in enumerate(devices):
        serial = dev.get_info(rs.camera_info.serial_number)
        name = dev.get_info(rs.camera_info.name)

        try:
            pipeline = rs.pipeline()
            config = rs.config()
            config.enable_device(serial)
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)

            pipeline.start(config)

            # Get a frame
            frames = pipeline.wait_for_frames(timeout_ms=5000)
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()

            if depth_frame and color_frame:
                print(f"  ✓ Camera {i+1} ({serial[:8]}...): OK")
            else:
                print(f"  ✗ Camera {i+1} ({serial[:8]}...): No frames")
                all_ok = False

            pipeline.stop()

        except Exception as e:
            print(f"  ✗ Camera {i+1} ({serial[:8]}...): Error - {e}")
            all_ok = False

    sys.exit(0 if all_ok else 1)

except Exception as e:
    print(f"ERROR: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
PYTHON_EOF

if python3 "$TEST_SCRIPT"; then
    echo -e "${GREEN}✓ All cameras responding correctly${NC}"
else
    echo -e "${RED}✗ Camera connectivity test failed${NC}"
    rm -f "$TEST_SCRIPT"
    exit 1
fi

rm -f "$TEST_SCRIPT"

echo ""
echo "=========================================="
echo "Camera health check complete!"
echo "=========================================="
