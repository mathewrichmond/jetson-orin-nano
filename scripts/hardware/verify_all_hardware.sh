#!/bin/bash
# Comprehensive Hardware Verification Script
# Verifies all planned hardware components for Isaac robot bench setup:
# 1. Two Realsense cameras (USB)
# 2. USB microphone
# 3. ODrive motor controller and accelerometer
# 4. iRobot developer kit serial connection (USB)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Track overall status
ALL_PASSED=true

echo "=========================================="
echo "Isaac Robot Hardware Verification"
echo "Bench Setup - All Hardware Components"
echo "=========================================="
echo ""

# Function to print section header
print_section() {
    echo ""
    echo -e "${BLUE}=== $1 ===${NC}"
}

# Function to print success
print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

# Function to print warning
print_warning() {
    echo -e "${YELLOW}⚠ $1${NC}"
    ALL_PASSED=false
}

# Function to print error
print_error() {
    echo -e "${RED}✗ $1${NC}"
    ALL_PASSED=false
}

# ============================================
# 1. RealSense Cameras (USB)
# ============================================
print_section "1. RealSense Cameras (USB)"

# Check if RealSense SDK is installed
if python3 -c "import pyrealsense2" 2>/dev/null; then
    RS_VERSION=$(python3 -c "import pyrealsense2; print(pyrealsense2.__version__)" 2>/dev/null || echo "unknown")
    print_success "RealSense SDK installed (version: $RS_VERSION)"
else
    print_error "RealSense SDK not installed"
    echo "  Run: sudo ./scripts/hardware/install_realsense.sh"
fi

# Check USB devices for RealSense cameras
REALSENSE_COUNT=$(lsusb | grep -i "8086:0b07\|8086:0b5c\|8086:0b64" | wc -l)
if [ "$REALSENSE_COUNT" -eq 2 ]; then
    print_success "Found 2 RealSense cameras"
    lsusb | grep -i "8086:0b07\|8086:0b5c\|8086:0b64" | while read line; do
        echo "    $line"
    done
elif [ "$REALSENSE_COUNT" -eq 1 ]; then
    print_warning "Found only 1 RealSense camera (expected 2)"
    lsusb | grep -i "8086:0b07\|8086:0b5c\|8086:0b64"
elif [ "$REALSENSE_COUNT" -eq 0 ]; then
    print_error "No RealSense cameras detected"
else
    print_warning "Found $REALSENSE_COUNT RealSense cameras (expected 2)"
fi

# Test camera connectivity
if python3 -c "import pyrealsense2" 2>/dev/null; then
    TEST_SCRIPT=$(mktemp)
    cat > "$TEST_SCRIPT" << 'PYTHON_EOF'
import pyrealsense2 as rs
import sys
ctx = rs.context()
devices = ctx.query_devices()
if len(devices) >= 2:
    print(f"SUCCESS: {len(devices)} cameras detected")
    sys.exit(0)
elif len(devices) == 1:
    print(f"WARNING: Only {len(devices)} camera detected")
    sys.exit(1)
else:
    print("ERROR: No cameras detected")
    sys.exit(1)
PYTHON_EOF
    if python3 "$TEST_SCRIPT" 2>/dev/null; then
        print_success "Camera connectivity test passed"
    else
        print_warning "Camera connectivity test failed"
    fi
    rm -f "$TEST_SCRIPT"
fi

# ============================================
# 2. USB Microphone
# ============================================
print_section "2. USB Microphone"

# Check for USB audio devices
USB_AUDIO_COUNT=$(lsusb | grep -i "audio\|microphone\|mic" | wc -l)
if [ "$USB_AUDIO_COUNT" -gt 0 ]; then
    print_success "Found $USB_AUDIO_COUNT USB audio device(s)"
    lsusb | grep -i "audio\|microphone\|mic" | while read line; do
        echo "    $line"
    done
else
    print_warning "No USB audio devices detected"
fi

# Check PulseAudio for audio devices
if command -v pactl &> /dev/null; then
    MIC_COUNT=$(pactl list sources short 2>/dev/null | grep -i "usb\|microphone\|mic" | wc -l)
    if [ "$MIC_COUNT" -gt 0 ]; then
        print_success "PulseAudio detected $MIC_COUNT microphone source(s)"
        pactl list sources short 2>/dev/null | grep -i "usb\|microphone\|mic" | while read line; do
            echo "    $line"
        done
    else
        print_warning "No USB microphone sources found in PulseAudio"
        echo "  Available sources:"
        pactl list sources short 2>/dev/null | head -5 | while read line; do
            echo "    $line"
        done
    fi
else
    print_warning "pactl not available (PulseAudio may not be installed)"
fi

# Test microphone recording capability
if command -v arecord &> /dev/null; then
    TEST_FILE=$(mktemp)
    if timeout 1 arecord -d 1 -f cd "$TEST_FILE" 2>/dev/null; then
        if [ -s "$TEST_FILE" ]; then
            print_success "Microphone recording test passed"
        else
            print_warning "Microphone recording test failed (no data captured)"
        fi
    else
        print_warning "Microphone recording test failed (arecord error)"
    fi
    rm -f "$TEST_FILE"
else
    print_warning "arecord not available (alsa-utils may not be installed)"
fi

# ============================================
# 3. ODrive Motor Controller and Accelerometer
# ============================================
print_section "3. ODrive Motor Controller and Accelerometer"

# Check for ODrive USB device (common VID/PID patterns)
ODRIVE_COUNT=$(lsusb | grep -i "odrive\|1209:0d32\|1209:0d33" | wc -l)
if [ "$ODRIVE_COUNT" -gt 0 ]; then
    print_success "Found $ODRIVE_COUNT ODrive device(s)"
    lsusb | grep -i "odrive\|1209:0d32\|1209:0d33" | while read line; do
        echo "    $line"
    done
else
    print_warning "No ODrive devices detected via USB"
    echo "  ODrive may be connected via CAN or serial"
fi

# Check for serial devices that might be ODrive
SERIAL_DEVICES=$(ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | wc -l)
if [ "$SERIAL_DEVICES" -gt 0 ]; then
    print_success "Found $SERIAL_DEVICES serial device(s) (may include ODrive)"
    ls /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | while read device; do
        echo "    $device"
    done
else
    print_warning "No serial devices found (/dev/ttyUSB*, /dev/ttyACM*)"
fi

# Check for accelerometer/IMU devices (I2C)
if command -v i2cdetect &> /dev/null; then
    I2C_BUSES=$(ls /dev/i2c-* 2>/dev/null | wc -l)
    if [ "$I2C_BUSES" -gt 0 ]; then
        print_success "Found $I2C_BUSES I2C bus(es) (accelerometer may be connected)"
        ls /dev/i2c-* 2>/dev/null | while read bus; do
            bus_num=$(echo "$bus" | grep -o '[0-9]*')
            echo "    $bus (bus $bus_num)"
        done
    else
        print_warning "No I2C buses found"
    fi
else
    print_warning "i2cdetect not available (i2c-tools may not be installed)"
fi

# Check for CAN interfaces (ODrive may use CAN)
if command -v ip &> /dev/null; then
    CAN_INTERFACES=$(ip link show 2>/dev/null | grep -i "can" | wc -l)
    if [ "$CAN_INTERFACES" -gt 0 ]; then
        print_success "Found CAN interface(s)"
        ip link show 2>/dev/null | grep -i "can" | while read line; do
            echo "    $line"
        done
    else
        print_warning "No CAN interfaces found"
    fi
fi

# ============================================
# 4. iRobot Developer Kit Serial Connection (USB)
# ============================================
print_section "4. iRobot Developer Kit Serial Connection (USB)"

# Check for iRobot USB devices (common patterns)
IROBOT_COUNT=$(lsusb | grep -i "irobot\|create\|roomba\|0bda:4014" | wc -l)
if [ "$IROBOT_COUNT" -gt 0 ]; then
    print_success "Found $IROBOT_COUNT iRobot device(s)"
    lsusb | grep -i "irobot\|create\|roomba\|0bda:4014" | while read line; do
        echo "    $line"
    done
else
    print_warning "No iRobot devices detected via USB"
    echo "  iRobot Create/Developer Kit may appear as a serial device"
fi

# Check for serial devices (iRobot typically uses /dev/ttyUSB* or /dev/ttyACM*)
if [ "$SERIAL_DEVICES" -gt 0 ]; then
    print_success "Serial devices available (may include iRobot)"
    echo "  Check /dev/ttyUSB* and /dev/ttyACM* for iRobot connection"
else
    print_warning "No serial devices found for iRobot connection"
fi

# Check serial port permissions
if groups | grep -q dialout; then
    print_success "User is in dialout group (serial port access OK)"
else
    print_warning "User not in dialout group"
    echo "  Run: sudo usermod -a -G dialout $USER"
    echo "  Then log out and back in"
fi

# ============================================
# Summary
# ============================================
echo ""
echo "=========================================="
if [ "$ALL_PASSED" = true ]; then
    echo -e "${GREEN}Hardware Verification Complete${NC}"
    echo -e "${GREEN}All hardware components detected!${NC}"
else
    echo -e "${YELLOW}Hardware Verification Complete${NC}"
    echo -e "${YELLOW}Some hardware components need attention${NC}"
fi
echo "=========================================="
echo ""

# Print detailed USB device list
echo "Full USB Device List:"
lsusb | while read line; do
    echo "  $line"
done

echo ""
echo "Serial Devices:"
if [ "$SERIAL_DEVICES" -gt 0 ]; then
    ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null | while read line; do
        echo "  $line"
    done
else
    echo "  None found"
fi

echo ""
echo "Next Steps:"
echo "  1. Verify all hardware is physically connected"
echo "  2. Check USB power supply (some devices need powered USB hub)"
echo "  3. Test each component individually"
echo "  4. Run ROS 2 nodes to verify software integration"
echo ""

exit 0
