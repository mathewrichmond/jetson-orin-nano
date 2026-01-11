#!/bin/bash
# PHAT Motor Controller Diagnostics
# Checks PHAT board connection, accelerometer, and GPIO access

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}PHAT Motor Controller Diagnostics${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Check I2C devices
echo -e "${GREEN}[1/5] Checking I2C buses...${NC}"
echo "Scanning I2C bus 1:"
i2cdetect -y 1 2>&1 | grep -E "UU|68|69|6a|6b" || echo "  No accelerometer detected at common addresses"
echo ""

# Check accelerometer detection
echo -e "${GREEN}[2/5] Testing accelerometer detection...${NC}"
python3 << 'PYTHON_EOF'
import sys
try:
    from smbus2 import SMBus

    bus = SMBus(1)
    addresses = {
        0x68: "MPU6050 (default)",
        0x69: "MPU6050 (AD0 high)",
        0x6A: "LSM6DS3",
        0x6B: "LSM6DS3 (alternative)",
        0x1D: "LSM303",
        0x53: "ADXL345"
    }

    found = False
    for addr, name in addresses.items():
        try:
            if addr == 0x68:  # MPU6050
                whoami = bus.read_byte_data(addr, 0x75)
                if whoami == 0x68:
                    print(f"  ✓ Found {name} at 0x{addr:02X} (WHO_AM_I=0x{whoami:02X})")
                    found = True
            elif addr == 0x6A:  # LSM6DS3
                whoami = bus.read_byte_data(addr, 0x0F)
                if whoami == 0x69:
                    print(f"  ✓ Found {name} at 0x{addr:02X} (WHO_AM_I=0x{whoami:02X})")
                    found = True
            else:
                # Try generic read
                val = bus.read_byte(addr)
                print(f"  ? Device at 0x{addr:02X} (may be {name}, value=0x{val:02X})")
                found = True
        except Exception:
            pass

    if not found:
        print("  ✗ No accelerometer detected at common addresses")
        print("  Check I2C connections and power")

    bus.close()
except ImportError:
    print("  ✗ smbus2 not installed. Install with: pip3 install smbus2")
except Exception as e:
    print(f"  ✗ Error: {e}")
PYTHON_EOF
echo ""

# Check accelerometer reading (orientation test)
echo -e "${GREEN}[3/5] Testing accelerometer reading (orientation check)...${NC}"
python3 << 'PYTHON_EOF'
import sys
import time
try:
    from smbus2 import SMBus

    bus = SMBus(1)
    addr = 0x68  # MPU6050 default

    try:
        # Wake up MPU6050
        bus.write_byte_data(addr, 0x6B, 0x00)
        time.sleep(0.1)

        # Read accelerometer data
        accel_data = bus.read_i2c_block_data(addr, 0x3B, 6)

        # Convert to signed 16-bit
        accel_x = (accel_data[0] << 8 | accel_data[1])
        accel_y = (accel_data[2] << 8 | accel_data[3])
        accel_z = (accel_data[4] << 8 | accel_data[5])

        if accel_x > 32767: accel_x -= 65536
        if accel_y > 32767: accel_y -= 65536
        if accel_z > 32767: accel_z -= 65536

        # Convert to m/s^2
        accel_x_ms2 = accel_x / 16384.0 * 9.81
        accel_y_ms2 = accel_y / 16384.0 * 9.81
        accel_z_ms2 = accel_z / 16384.0 * 9.81

        print(f"  Accelerometer readings:")
        print(f"    X: {accel_x_ms2:+.2f} m/s²")
        print(f"    Y: {accel_y_ms2:+.2f} m/s²")
        print(f"    Z: {accel_z_ms2:+.2f} m/s²")
        print(f"")

        # Check orientation (gravity should be ~9.81 m/s² in one axis)
        total_g = (accel_x_ms2**2 + accel_y_ms2**2 + accel_z_ms2**2)**0.5
        print(f"  Total acceleration magnitude: {total_g:.2f} m/s²")

        if 8.0 < total_g < 11.0:
            print(f"  ✓ Gravity detected! Board appears to be working.")

            # Determine orientation
            abs_x = abs(accel_x_ms2)
            abs_y = abs(accel_y_ms2)
            abs_z = abs(accel_z_ms2)

            if abs_z > abs_x and abs_z > abs_y:
                print(f"  Orientation: Z-axis dominant (board likely flat)")
            elif abs_y > abs_x and abs_y > abs_z:
                print(f"  Orientation: Y-axis dominant")
            elif abs_x > abs_y and abs_x > abs_z:
                print(f"  Orientation: X-axis dominant")
        else:
            print(f"  ⚠ Unexpected acceleration magnitude (expected ~9.81 m/s²)")
            print(f"  Board may be moving or accelerometer needs calibration")

    except Exception as e:
        print(f"  ✗ Error reading accelerometer: {e}")
        print(f"  Check I2C address and connections")

    bus.close()
except Exception as e:
    print(f"  ✗ Error: {e}")
PYTHON_EOF
echo ""

# Check GPIO access
echo -e "${GREEN}[4/5] Checking GPIO access...${NC}"
python3 << 'PYTHON_EOF'
import sys
GPIO_AVAILABLE = False
try:
    import Jetson.GPIO as GPIO
    GPIO_AVAILABLE = True
    print("  ✓ Jetson.GPIO available")
except ImportError:
    try:
        import RPi.GPIO as GPIO
        GPIO_AVAILABLE = True
        print("  ✓ RPi.GPIO available")
    except ImportError:
        print("  ✗ No GPIO library available")
        print("  Install Jetson.GPIO for Jetson boards")

if GPIO_AVAILABLE:
    try:
        GPIO.setmode(GPIO.BOARD)
        print("  ✓ GPIO mode set successfully")

        # Test a GPIO pin (don't actually use it, just check access)
        test_pin = 18
        GPIO.setup(test_pin, GPIO.OUT)
        GPIO.output(test_pin, GPIO.LOW)
        GPIO.cleanup(test_pin)
        print(f"  ✓ GPIO pin {test_pin} accessible")
    except Exception as e:
        print(f"  ✗ GPIO access error: {e}")
        print("  Check user groups: groups | grep gpio")
PYTHON_EOF
echo ""

# Check ROS 2 node status
echo -e "${GREEN}[5/5] Checking ROS 2 node status...${NC}"
if command -v ros2 &> /dev/null; then
    source /opt/ros/humble/setup.bash 2>/dev/null || true
    source ~/ros2_ws/install/setup.bash 2>/dev/null || true

    if ros2 node list 2>&1 | grep -q phat; then
        echo "  ✓ PHAT node is running"

        echo "  Status topic:"
        timeout 2 ros2 topic echo /phat/status --once 2>&1 | grep "data:" | head -1 || echo "    (no data yet)"

        echo "  IMU topic:"
        timeout 2 ros2 topic echo /phat/imu --once 2>&1 | grep -E "x:|y:|z:" | head -3 || echo "    (no data yet)"
    else
        echo "  ⚠ PHAT node not running"
        echo "  Start with: ./scripts/system/manage_graph.sh restart robot"
    fi
else
    echo "  ⚠ ROS 2 not available"
fi

echo ""
echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Diagnostics Complete${NC}"
echo -e "${BLUE}========================================${NC}"
