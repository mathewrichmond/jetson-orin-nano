#!/bin/bash
# PCA9685 Servo Controller Diagnostics
# Systematic testing and troubleshooting for SparkFun Auto pHAT servo controller

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m'

# Configuration
I2C_BUS=7  # Jetson Orin Nano GPIO I2C bus
PCA9685_DEFAULT_ADDR=0x40
PCA9685_ADDR_RANGE_START=0x40
PCA9685_ADDR_RANGE_END=0x47

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}PCA9685 Servo Controller Diagnostics${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Check if running with sudo (needed for some operations)
if [ "$EUID" -ne 0 ]; then
    echo -e "${YELLOW}Note: Some tests may require sudo. Run with 'sudo' if tests fail.${NC}"
    echo ""
fi

# [1/10] Check I2C bus exists
echo -e "${GREEN}[1/10] Checking I2C bus availability...${NC}"
if [ -e "/dev/i2c-$I2C_BUS" ]; then
    echo -e "  ✓ I2C bus $I2C_BUS exists: /dev/i2c-$I2C_BUS"
else
    echo -e "  ${RED}✗ I2C bus $I2C_BUS not found${NC}"
    echo "  Available buses:"
    ls -1 /dev/i2c-* 2>/dev/null || echo "    No I2C buses found"
    echo ""
    echo "  Troubleshooting:"
    echo "    - Verify I2C is enabled in device tree"
    echo "    - Check that GPIO I2C is configured"
    echo "    - On Jetson, GPIO I2C is typically bus 7"
    exit 1
fi
echo ""

# [2/10] Check I2C tools
echo -e "${GREEN}[2/10] Checking I2C tools...${NC}"
if command -v i2cdetect &> /dev/null && command -v i2cget &> /dev/null && command -v i2cset &> /dev/null; then
    echo "  ✓ i2c-tools installed (i2cdetect, i2cget, i2cset)"
else
    echo -e "  ${RED}✗ i2c-tools not found${NC}"
    echo "  Install with: sudo apt-get install i2c-tools"
    exit 1
fi
echo ""

# [3/10] Scan I2C bus
echo -e "${GREEN}[3/10] Scanning I2C bus $I2C_BUS...${NC}"
echo "  Full bus scan:"
i2cdetect -y $I2C_BUS 2>&1 | sed 's/^/    /'
echo ""

# Extract detected addresses
DETECTED_ADDRESSES=$(i2cdetect -y $I2C_BUS | grep -E '^[0-9a-f]+:' | sed 's/^[0-9a-f]*: //' | grep -oE '[0-9a-f]{2}' | grep -v '^--$' || true)

if [ -z "$DETECTED_ADDRESSES" ]; then
    echo -e "  ${RED}✗ No devices detected on I2C bus $I2C_BUS${NC}"
    echo ""
    echo "  This indicates a serious issue:"
    echo "    - Wrong I2C bus number (try bus 1 for Raspberry Pi)"
    echo "    - Pull-up resistors missing or incorrect"
    echo "    - Power not connected to Auto pHAT"
    echo "    - Hardware failure"
    echo ""
    echo "  See: docs/hardware/sparkfun-auto-phat/I2C_TROUBLESHOOTING.md"
    exit 1
else
    echo "  Detected addresses:"
    for addr in $DETECTED_ADDRESSES; do
        echo "    - 0x$addr"
    done
fi
echo ""

# [4/10] Check for PCA9685 at default address
echo -e "${GREEN}[4/10] Checking for PCA9685 at default address 0x${PCA9685_DEFAULT_ADDR:2}...${NC}"
if echo "$DETECTED_ADDRESSES" | grep -qi "${PCA9685_DEFAULT_ADDR:2}"; then
    echo -e "  ${GREEN}✓ Device found at 0x${PCA9685_DEFAULT_ADDR:2}${NC}"
    PCA9685_ADDR=$PCA9685_DEFAULT_ADDR
else
    echo -e "  ${RED}✗ No device at default address 0x${PCA9685_DEFAULT_ADDR:2}${NC}"
    PCA9685_ADDR=""
fi
echo ""

# [5/10] Scan all possible PCA9685 addresses
echo -e "${GREEN}[5/10] Scanning PCA9685 address range (0x40-0x47)...${NC}"
echo "  Checking addresses 0x40 through 0x47:"
FOUND_AT_ALT=false
for ((addr=$PCA9685_ADDR_RANGE_START; addr<=$PCA9685_ADDR_RANGE_END; addr++)); do
    addr_hex=$(printf "0x%02x" $addr)
    if echo "$DETECTED_ADDRESSES" | grep -qi "$(printf "%02x" $addr)"; then
        echo -e "    ${GREEN}✓ Found at $addr_hex${NC}"
        if [ -z "$PCA9685_ADDR" ]; then
            PCA9685_ADDR=$addr_hex
            FOUND_AT_ALT=true
        fi
    else
        echo "    - Not at $addr_hex"
    fi
done

if [ -n "$PCA9685_ADDR" ] && [ "$FOUND_AT_ALT" = true ]; then
    echo ""
    echo -e "  ${YELLOW}Note: PCA9685 found at non-default address${NC}"
    echo "  Check SRV ADD jumpers on back of board"
    echo "  See: docs/hardware/sparkfun-auto-phat/JUMPER_CONFIGURATION.md"
fi
echo ""

# [6/10] Attempt software reset
echo -e "${GREEN}[6/10] Attempting PCA9685 software reset...${NC}"
echo "  Sending SWRST command (General Call address 0x00, data 0x06)..."
if i2cset -y $I2C_BUS 0x00 0x06 2>&1 | grep -q "Error"; then
    echo -e "  ${YELLOW}⚠ Software reset command may have failed${NC}"
    echo "    (This is normal if no PCA9685 devices are present)"
else
    echo "  ✓ Reset command sent"
    sleep 0.5
    
    # Rescan after reset
    echo "  Rescanning bus..."
    DETECTED_ADDRESSES_AFTER=$(i2cdetect -y $I2C_BUS | grep -E '^[0-9a-f]+:' | sed 's/^[0-9a-f]*: //' | grep -oE '[0-9a-f]{2}' | grep -v '^--$' || true)
    
    # Check if PCA9685 appeared
    if [ -z "$PCA9685_ADDR" ]; then
        for ((addr=$PCA9685_ADDR_RANGE_START; addr<=$PCA9685_ADDR_RANGE_END; addr++)); do
            addr_hex=$(printf "%02x" $addr)
            if echo "$DETECTED_ADDRESSES_AFTER" | grep -qi "$addr_hex"; then
                PCA9685_ADDR=$(printf "0x%02x" $addr)
                echo -e "  ${GREEN}✓ PCA9685 appeared at $PCA9685_ADDR after reset!${NC}"
                break
            fi
        done
    fi
fi
echo ""

# [7/10] Test PCA9685 register access
if [ -n "$PCA9685_ADDR" ]; then
    echo -e "${GREEN}[7/10] Testing PCA9685 register access at $PCA9685_ADDR...${NC}"
    
    # Read MODE1 register (0x00)
    echo "  Reading MODE1 register (0x00)..."
    MODE1=$(i2cget -y $I2C_BUS $PCA9685_ADDR 0x00 2>&1)
    if echo "$MODE1" | grep -q "Error"; then
        echo -e "  ${RED}✗ Failed to read MODE1 register${NC}"
        echo "    Error: $MODE1"
    else
        echo "  ✓ MODE1 = $MODE1"
        
        # Decode MODE1 bits
        MODE1_VAL=$(printf "%d" "$MODE1")
        echo "    Bit 7 (RESTART): $((MODE1_VAL >> 7 & 1))"
        echo "    Bit 6 (EXTCLK):  $((MODE1_VAL >> 6 & 1))"
        echo "    Bit 5 (AI):      $((MODE1_VAL >> 5 & 1)) (Auto-increment)"
        echo "    Bit 4 (SLEEP):   $((MODE1_VAL >> 4 & 1)) (1=sleep, 0=normal)"
    fi
    
    # Read MODE2 register (0x01)
    echo ""
    echo "  Reading MODE2 register (0x01)..."
    MODE2=$(i2cget -y $I2C_BUS $PCA9685_ADDR 0x01 2>&1)
    if echo "$MODE2" | grep -q "Error"; then
        echo -e "  ${RED}✗ Failed to read MODE2 register${NC}"
    else
        echo "  ✓ MODE2 = $MODE2"
    fi
    
    # Read PRESCALE register (0xFE)
    echo ""
    echo "  Reading PRESCALE register (0xFE)..."
    PRESCALE=$(i2cget -y $I2C_BUS $PCA9685_ADDR 0xFE 2>&1)
    if echo "$PRESCALE" | grep -q "Error"; then
        echo -e "  ${YELLOW}⚠ Could not read PRESCALE (chip may be in sleep mode)${NC}"
    else
        PRESCALE_VAL=$(printf "%d" "$PRESCALE")
        FREQ=$((25000000 / (4096 * ($PRESCALE_VAL + 1))))
        echo "  ✓ PRESCALE = $PRESCALE (value: $PRESCALE_VAL)"
        echo "    Calculated PWM frequency: ~${FREQ} Hz"
        if [ $FREQ -ge 45 ] && [ $FREQ -le 55 ]; then
            echo "    (Configured for servos: 50 Hz nominal)"
        fi
    fi
else
    echo -e "${GREEN}[7/10] Testing PCA9685 register access...${NC}"
    echo -e "  ${RED}✗ Skipped - PCA9685 not detected${NC}"
fi
echo ""

# [8/10] Check other Auto pHAT devices
echo -e "${GREEN}[8/10] Checking other Auto pHAT devices (for comparison)...${NC}"

# IMU at 0x69
if echo "$DETECTED_ADDRESSES" | grep -qi "69"; then
    echo "  ✓ IMU (ICM-20948) found at 0x69"
    IMU_WHOAMI=$(i2cget -y $I2C_BUS 0x69 0x00 2>&1)
    if echo "$IMU_WHOAMI" | grep -q "0x"; then
        echo "    WHO_AM_I register: $IMU_WHOAMI (should be 0xEA)"
    fi
elif echo "$DETECTED_ADDRESSES" | grep -qi "68"; then
    echo "  ✓ IMU (ICM-20948) found at 0x68 (non-default)"
else
    echo "  ✗ IMU not found at 0x68 or 0x69"
fi

# Motor driver at 0x5D
if echo "$DETECTED_ADDRESSES" | grep -qi "5d"; then
    echo "  ✓ Motor Driver (PSoC 4245) found at 0x5D"
else
    echo "  - Motor Driver not detected (may be at different address)"
fi

# Encoder reader at 0x73
if echo "$DETECTED_ADDRESSES" | grep -qi "73"; then
    echo "  ✓ Encoder Reader (ATtiny84) found at 0x73"
else
    echo "  - Encoder Reader not detected"
fi

echo ""
echo "  Analysis:"
if echo "$DETECTED_ADDRESSES" | grep -qi "69\|68"; then
    echo "    ✓ At least one Auto pHAT device working (IMU)"
    echo "    → I2C bus is functional"
    echo "    → Issue is specific to PCA9685"
else
    echo "    ⚠ No Auto pHAT devices detected"
    echo "    → Problem may be board-wide (power, connection, I2C bus)"
fi
echo ""

# [9/10] Power and hardware checks
echo -e "${GREEN}[9/10] Hardware check recommendations...${NC}"
if [ -z "$PCA9685_ADDR" ]; then
    echo "  PCA9685 not detected. Check the following:"
    echo ""
    echo "  1. Power Supply:"
    echo "     - USB-C connected to Auto pHAT?"
    echo "     - 5V rail active? (measure VDD pin on PCA9685)"
    echo ""
    echo "  2. Output Enable (/OE) Pin:"
    echo "     - PCA9685 pin 23 must be LOW (connected to GND)"
    echo "     - Check schematic: docs/hardware/sparkfun-auto-phat/datasheets/SparkFun_Auto_pHAT_Schematic.pdf"
    echo ""
    echo "  3. Address Jumpers:"
    echo "     - Check SRV ADD jumpers on back of board"
    echo "     - All open = 0x40 (default)"
    echo "     - See: docs/hardware/sparkfun-auto-phat/JUMPER_CONFIGURATION.md"
    echo ""
    echo "  4. Pull-up Resistors:"
    echo "     - Verify no pull-DOWN resistors on I2C lines"
    echo "     - SDA/SCL should read ~3.3V when idle"
    echo "     - See: docs/hardware/sparkfun-auto-phat/I2C_TROUBLESHOOTING.md"
    echo ""
    echo "  5. Hardware Damage:"
    echo "     - Check for physical damage to PCA9685 IC"
    echo "     - Look for burned components or solder bridges"
    echo "     - Test with multimeter for shorts"
else
    echo "  ✓ PCA9685 detected and responding"
    echo ""
    echo "  Next steps to test servo operation:"
    echo "    1. Connect a servo to channel 0"
    echo "    2. Run servo test script: ./scripts/hardware/test_servo_hardware.sh"
    echo "    3. Check ROS 2 node: ros2 topic echo /phat/camera_pan"
fi
echo ""

# [10/10] Summary and next steps
echo -e "${GREEN}[10/10] Diagnostic Summary${NC}"
echo ""
if [ -n "$PCA9685_ADDR" ]; then
    echo -e "  ${GREEN}✓ SUCCESS: PCA9685 found at address $PCA9685_ADDR${NC}"
    echo ""
    echo "  Status:"
    echo "    - Device is powered and responding"
    echo "    - I2C communication working"
    echo "    - Ready for servo control testing"
    echo ""
    echo "  Configuration:"
    echo "    - Update config/hardware/phat_params.yaml if address is not 0x40"
    echo "    - Set: servo_i2c_address: $PCA9685_ADDR"
else
    echo -e "  ${RED}✗ FAILURE: PCA9685 not detected${NC}"
    echo ""
    echo "  Likely causes (in order of probability):"
    echo "    1. Output Enable (/OE) pin not asserted (pin 23 must be LOW)"
    echo "    2. Power supply issue (VDD not reaching PCA9685)"
    echo "    3. Address jumpers configured incorrectly"
    echo "    4. Hardware damage to PCA9685 IC"
    echo "    5. Incorrect I2C bus number"
    echo ""
    echo "  Recommended actions:"
    echo "    1. Read troubleshooting guide:"
    echo "       docs/hardware/sparkfun-auto-phat/I2C_TROUBLESHOOTING.md"
    echo ""
    echo "    2. Check schematic for /OE pin connection:"
    echo "       docs/hardware/sparkfun-auto-phat/datasheets/SparkFun_Auto_pHAT_Schematic.pdf"
    echo ""
    echo "    3. Measure voltages with multimeter:"
    echo "       - PCA9685 VDD (pin 28): should be ~5V"
    echo "       - PCA9685 /OE (pin 23): should be ~0V (LOW)"
    echo "       - SDA/SCL: should be ~3.3V when idle"
    echo ""
    echo "    4. Verify jumper configuration:"
    echo "       docs/hardware/sparkfun-auto-phat/JUMPER_CONFIGURATION.md"
fi

echo ""
echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Diagnostics Complete${NC}"
echo -e "${BLUE}========================================${NC}"

# Exit with appropriate code
if [ -n "$PCA9685_ADDR" ]; then
    exit 0
else
    exit 1
fi
