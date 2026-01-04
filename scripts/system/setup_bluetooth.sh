#!/bin/bash
# Setup Bluetooth
# Installs and configures Bluetooth support

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m'

echo "=========================================="
echo "Bluetooth Setup"
echo "=========================================="
echo ""

# Check if running as root for some operations
NEED_ROOT=false
if [ "$EUID" -ne 0 ]; then
    NEED_ROOT=true
    echo -e "${YELLOW}Note: Some operations require root privileges${NC}"
fi

# Step 1: Install Bluetooth packages
echo -e "${BLUE}[1/4] Installing Bluetooth packages...${NC}"

BLUETOOTH_PACKAGES=(
    "bluez"
    "bluez-tools"
    "bluetooth"
    "pulseaudio-module-bluetooth"
)

MISSING_PACKAGES=()
for pkg in "${BLUETOOTH_PACKAGES[@]}"; do
    if ! dpkg -l | grep -q "^ii.*$pkg"; then
        MISSING_PACKAGES+=("$pkg")
    fi
done

if [ ${#MISSING_PACKAGES[@]} -gt 0 ]; then
    echo "Installing: ${MISSING_PACKAGES[*]}"
    if [ "$NEED_ROOT" = true ]; then
        sudo apt-get update
        sudo apt-get install -y "${MISSING_PACKAGES[@]}"
    else
        apt-get update
        apt-get install -y "${MISSING_PACKAGES[@]}"
    fi
    echo -e "${GREEN}✓ Bluetooth packages installed${NC}"
else
    echo -e "${GREEN}✓ Bluetooth packages already installed${NC}"
fi

# Step 2: Enable Bluetooth service
echo ""
echo -e "${BLUE}[2/4] Enabling Bluetooth service...${NC}"

if [ "$NEED_ROOT" = true ]; then
    sudo systemctl enable bluetooth
    sudo systemctl start bluetooth
else
    systemctl enable bluetooth
    systemctl start bluetooth
fi

sleep 2

if systemctl is-active --quiet bluetooth; then
    echo -e "${GREEN}✓ Bluetooth service enabled and running${NC}"
else
    echo -e "${YELLOW}⚠ Bluetooth service may need manual start${NC}"
fi

# Step 3: Check Bluetooth hardware
echo ""
echo -e "${BLUE}[3/4] Checking Bluetooth hardware...${NC}"

if command -v hciconfig &> /dev/null; then
    BT_DEVICES=$(hciconfig 2>/dev/null | grep -E "^hci" | wc -l)
    if [ "$BT_DEVICES" -gt 0 ]; then
        echo -e "${GREEN}✓ Bluetooth adapter(s) found: $BT_DEVICES${NC}"
        hciconfig | grep -E "^hci|UP|DOWN" | head -10
    else
        echo -e "${YELLOW}⚠ No Bluetooth adapters found${NC}"
    fi
else
    echo -e "${YELLOW}⚠ hciconfig not available${NC}"
fi

# Check with bluetoothctl
if command -v bluetoothctl &> /dev/null; then
    echo ""
    echo "Bluetooth controller status:"
    bluetoothctl show 2>/dev/null | head -5 || echo "  (Controller may need to be powered on)"
fi

# Step 4: Configure Bluetooth
echo ""
echo -e "${BLUE}[4/4] Configuring Bluetooth...${NC}"

# Enable discoverable mode (optional, can be changed)
if [ "$NEED_ROOT" = true ]; then
    # Add user to bluetooth group if not already
    if ! groups $USER | grep -q bluetooth; then
        echo "Adding user to bluetooth group..."
        sudo usermod -a -G bluetooth $USER
        echo -e "${GREEN}✓ User added to bluetooth group${NC}"
        echo -e "${YELLOW}⚠ Log out and back in for group changes to take effect${NC}"
    else
        echo -e "${GREEN}✓ User already in bluetooth group${NC}"
    fi
else
    if ! groups $USER | grep -q bluetooth; then
        usermod -a -G bluetooth $USER
        echo -e "${GREEN}✓ User added to bluetooth group${NC}"
    fi
fi

# Configure Bluetooth to be discoverable (optional)
# This can be changed via bluetoothctl or GUI later
echo ""
echo "Bluetooth configuration options:"
echo "  - Make discoverable: bluetoothctl discoverable on"
echo "  - Make pairable: bluetoothctl pairable on"
echo "  - Scan for devices: bluetoothctl scan on"
echo "  - Pair device: bluetoothctl pair <MAC>"
echo "  - Connect device: bluetoothctl connect <MAC>"

# Summary
echo ""
echo -e "${GREEN}=========================================="
echo "Bluetooth Setup Complete!"
echo "==========================================${NC}"
echo ""
echo "Bluetooth Status:"
if systemctl is-active --quiet bluetooth; then
    echo "  Service: ✓ Running"
else
    echo "  Service: ✗ Not running"
fi

if command -v bluetoothctl &> /dev/null; then
    echo ""
    echo "Useful commands:"
    echo "  bluetoothctl                    - Interactive Bluetooth control"
    echo "  bluetoothctl power on           - Power on Bluetooth"
    echo "  bluetoothctl scan on            - Scan for devices"
    echo "  bluetoothctl devices            - List known devices"
    echo "  bluetoothctl pair <MAC>         - Pair with device"
    echo "  bluetoothctl connect <MAC>     - Connect to device"
    echo ""
    echo "Check status:"
    echo "  systemctl status bluetooth"
    echo "  hciconfig"
    echo "  bluetoothctl show"
fi

echo ""
