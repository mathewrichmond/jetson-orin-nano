#!/bin/bash
# Test servo hardware (PCA9685) connectivity

set -e

echo "=========================================="
echo "Servo Hardware Diagnostic"
echo "=========================================="

# Check I2C bus 7
echo ""
echo "1. Checking I2C bus 7..."
i2cdetect -y 7

# Test PCA9685 access
echo ""
echo "2. Testing PCA9685 access at 0x40..."
python3 << 'PYTHON'
from smbus2 import SMBus
import sys

try:
    bus = SMBus(7)
    addr = 0x40
    
    # Try to read MODE1 register
    try:
        mode1 = bus.read_byte_data(addr, 0x00)
        print(f"✓ PCA9685 detected! MODE1: 0x{mode1:02X}")
        
        mode2 = bus.read_byte_data(addr, 0x01)
        print(f"✓ MODE2: 0x{mode2:02X}")
        print("✓ PCA9685 is accessible and responding")
        sys.exit(0)
    except Exception as e:
        print(f"✗ Cannot read from PCA9685: {e}")
        print("  Possible causes:")
        print("  - PCA9685 not powered")
        print("  - I2C address incorrect")
        print("  - Hardware not connected")
        print("  - I2C bus issue")
        sys.exit(1)
    
    bus.close()
except Exception as e:
    print(f"✗ Cannot open I2C bus 7: {e}")
    sys.exit(1)
PYTHON

# Check ROS 2 node status
echo ""
echo "3. Checking PHAT node status..."
if ros2 node list 2>/dev/null | grep -q phat; then
    echo "✓ PHAT node is running"
    ros2 topic echo /phat/status --once 2>/dev/null || echo "  (status topic not available)"
else
    echo "✗ PHAT node is not running"
fi

echo ""
echo "=========================================="
echo "Diagnostic complete"
echo "=========================================="
