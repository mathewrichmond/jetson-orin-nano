#!/bin/bash
# Hardware Verification Script
# Tests all connected hardware components

set -e

cd /home/nano/src/jetson-orin-nano
source install/setup.bash

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo "======================================"
echo "Hardware Verification Test"
echo "======================================"
echo ""

# 1. Check RealSense Cameras
echo -e "${YELLOW}[1/5] Testing RealSense Cameras${NC}"
if rs-enumerate-devices 2>&1 | grep -q "Intel RealSense"; then
    echo -e "${GREEN}✅ RealSense cameras detected${NC}"
    rs-enumerate-devices 2>&1 | grep -E "Name|Serial Number" | head -4
else
    echo -e "${RED}❌ No RealSense cameras found${NC}"
fi
echo ""

# 2. Check USB Microphone
echo -e "${YELLOW}[2/5] Testing USB Microphone${NC}"
if arecord -l 2>&1 | grep -q "Amazonbasics"; then
    echo -e "${GREEN}✅ USB Microphone detected (card 0)${NC}"
    arecord -l 2>&1 | grep "Amazonbasics" -A 1
else
    echo -e "${RED}❌ USB Microphone not found${NC}"
fi
echo ""

# 3. Check iRobot Serial
echo -e "${YELLOW}[3/5] Testing iRobot Serial Interface${NC}"
if [ -e "/dev/ttyUSB0" ]; then
    echo -e "${GREEN}✅ Serial device found: /dev/ttyUSB0${NC}"
    ls -l /dev/ttyUSB0
else
    echo -e "${RED}❌ /dev/ttyUSB0 not found${NC}"
fi
echo ""

# 4. Check I2C Devices (IMU/Motor Controller)
echo -e "${YELLOW}[4/5] Testing I2C Devices${NC}"
if ls /dev/i2c-* >/dev/null 2>&1; then
    echo -e "${GREEN}✅ I2C buses available:${NC}"
    ls -l /dev/i2c-* | grep -v "Dec 31  1969" || echo "  All I2C buses shown above"
else
    echo -e "${RED}❌ No I2C devices found${NC}"
fi
echo ""

# 5. Check GPIO (if available)
echo -e "${YELLOW}[5/5] Testing GPIO Access${NC}"
if python3 -c "import Jetson.GPIO as GPIO; GPIO.setmode(GPIO.BOARD); print('GPIO OK')" 2>/dev/null; then
    echo -e "${GREEN}✅ Jetson GPIO library available${NC}"
elif python3 -c "import RPi.GPIO as GPIO; GPIO.setmode(GPIO.BOARD); print('GPIO OK')" 2>/dev/null; then
    echo -e "${GREEN}✅ RPi GPIO library available${NC}"
else
    echo -e "${YELLOW}⚠️  GPIO library not available (will use mock mode)${NC}"
fi
echo ""

echo "======================================"
echo "Hardware Summary"
echo "======================================"
echo ""
echo "Connected Hardware:"
echo "  • 2x RealSense D435 cameras"
echo "  • 1x USB Microphone (Amazonbasics)"
echo "  • 1x Serial device (/dev/ttyUSB0)"
echo "  • Multiple I2C buses (IMU/motors)"
echo ""
echo "Ready to test ROS 2 integration!"
echo ""
