#!/bin/bash
# Setup USB-C Display and Dock Support
# Configures USB-C DisplayPort Alt Mode and dock functionality for Jetson Orin Nano

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

echo "=========================================="
echo "USB-C Display & Dock Setup"
echo "=========================================="
echo ""

# Check if running as root for some operations
if [ "$EUID" -ne 0 ]; then
    echo -e "${YELLOW}Note: Some operations require root privileges${NC}"
    echo "Run with sudo for full functionality"
fi

# 1. Check current display status
echo -e "${BLUE}[1/6] Checking display status...${NC}"
echo ""
echo "Available DRM devices:"
ls -1 /sys/class/drm/card*/status 2>/dev/null | while read status_file; do
    card=$(basename $(dirname $status_file))
    status=$(cat $status_file 2>/dev/null || echo "unknown")
    echo "  $card: $status"
done

echo ""
echo "Display connectors:"
ls -1 /sys/class/drm/card*/status 2>/dev/null | sed 's|/status||' | while read connector; do
    name=$(basename $connector)
    status=$(cat $connector/status 2>/dev/null || echo "unknown")
    enabled=$(cat $connector/enabled 2>/dev/null || echo "unknown")
    echo "  $name: status=$status, enabled=$enabled"
done

# 2. Check USB-C ports
echo ""
echo -e "${BLUE}[2/6] Checking USB-C ports...${NC}"
echo ""
if command -v lsusb &> /dev/null; then
    echo "USB devices:"
    lsusb | grep -i "type\|usb" || echo "  (No USB-C specific devices found)"
else
    echo "lsusb not available"
fi

# 3. Check for DisplayPort Alt Mode support
echo ""
echo -e "${BLUE}[3/6] Checking DisplayPort Alt Mode support...${NC}"
echo ""
if [ -d "/sys/class/drm" ]; then
    DP_CONNECTORS=$(find /sys/class/drm -name "*-DP-*" -o -name "card*-DP-*" 2>/dev/null | wc -l)
    echo "DisplayPort connectors found: $DP_CONNECTORS"
    if [ "$DP_CONNECTORS" -gt 0 ]; then
        echo -e "${GREEN}✓ DisplayPort support detected${NC}"
        find /sys/class/drm -name "*-DP-*" -o -name "card*-DP-*" 2>/dev/null | while read dp; do
            echo "  $(basename $dp)"
        done
    else
        echo -e "${YELLOW}⚠ No DisplayPort connectors found${NC}"
    fi
else
    echo -e "${RED}✗ DRM subsystem not available${NC}"
fi

# 4. Install/check required packages
echo ""
echo -e "${BLUE}[4/6] Checking required packages...${NC}"
echo ""

REQUIRED_PACKAGES=(
    "xserver-xorg-core"
    "x11-xserver-utils"
    "x11-utils"
)

MISSING_PACKAGES=()
for pkg in "${REQUIRED_PACKAGES[@]}"; do
    if ! dpkg -l | grep -q "^ii.*$pkg"; then
        MISSING_PACKAGES+=("$pkg")
    fi
done

if [ ${#MISSING_PACKAGES[@]} -gt 0 ]; then
    echo -e "${YELLOW}Missing packages: ${MISSING_PACKAGES[*]}${NC}"
    echo "Install with: sudo apt install ${MISSING_PACKAGES[*]}"
else
    echo -e "${GREEN}✓ Required packages installed${NC}"
fi

# 5. Check kernel modules
echo ""
echo -e "${BLUE}[5/6] Checking kernel modules...${NC}"
echo ""

REQUIRED_MODULES=(
    "drm"
    "drm_kms_helper"
    "typec"
    "usbcore"
)

for module in "${REQUIRED_MODULES[@]}"; do
    if lsmod | grep -q "^${module}"; then
        echo -e "${GREEN}✓ Module $module loaded${NC}"
    else
        echo -e "${YELLOW}⚠ Module $module not loaded (may be built-in)${NC}"
    fi
done

# 6. Display configuration recommendations
echo ""
echo -e "${BLUE}[6/6] Configuration recommendations...${NC}"
echo ""

echo "For USB-C Display/Dock support on Jetson Orin Nano:"
echo ""
echo "1. USB-C Port Limitations:"
echo "   - Jetson Orin Nano USB-C ports may not support DP Alt Mode"
echo "   - Check if your dock uses DisplayLink (software-based)"
echo ""
echo "2. Alternative Display Outputs:"
echo "   - Use HDMI port if available"
echo "   - Use DisplayPort via adapter if available"
echo ""
echo "3. DisplayLink Docks:"
echo "   - If using DisplayLink dock, install DisplayLink driver:"
echo "     sudo apt install displaylink-driver"
echo ""
echo "4. Manual Display Configuration:"
echo "   - Use xrandr to configure displays:"
echo "     xrandr --output <connector> --auto --primary"
echo ""
echo "5. Check Display Status:"
echo "   - Monitor: watch -n 1 'cat /sys/class/drm/card*/status'"
echo "   - Connectors: ls -la /sys/class/drm/card*/"
echo ""

# Create helper script for display configuration
cat > "$PROJECT_ROOT/scripts/hardware/configure_display.sh" << 'HELPER_EOF'
#!/bin/bash
# Quick display configuration helper

echo "Available displays:"
xrandr --listmonitors 2>/dev/null || echo "X server not running or no displays detected"

echo ""
echo "Display connectors:"
for connector in /sys/class/drm/card*/status; do
    if [ -f "$connector" ]; then
        name=$(basename $(dirname $connector))
        status=$(cat $connector)
        echo "  $name: $status"
    fi
done

echo ""
echo "To configure a display:"
echo "  xrandr --output <connector> --auto --primary"
echo ""
echo "To enable a connector:"
echo "  echo on > /sys/class/drm/card*/<connector>/dpms"
HELPER_EOF

chmod +x "$PROJECT_ROOT/scripts/hardware/configure_display.sh"

echo -e "${GREEN}✓ Created display configuration helper: scripts/hardware/configure_display.sh${NC}"
echo ""
echo "=========================================="
echo "Setup Complete"
echo "=========================================="
echo ""
echo "Next steps:"
echo "1. Connect your USB-C monitor/dock"
echo "2. Check if detected: ./scripts/hardware/configure_display.sh"
echo "3. If using DisplayLink dock, install driver: sudo apt install displaylink-driver"
echo "4. Configure display: xrandr --output <connector> --auto"
echo ""
