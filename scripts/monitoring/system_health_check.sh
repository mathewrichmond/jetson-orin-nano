#!/bin/bash
# System Health Check Script
# Performs comprehensive system health checks for the Isaac robot system

set -e

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo "=========================================="
echo "Isaac System Health Check"
echo "=========================================="
echo ""

# Check system hostname
echo -e "${GREEN}Checking hostname...${NC}"
if [ "$(hostname)" == "isaac" ]; then
    echo "✓ Hostname is set to 'isaac'"
else
    echo -e "${YELLOW}⚠ Hostname is '$(hostname)', expected 'isaac'${NC}"
fi

# Check ROS 2 installation
echo -e "${GREEN}Checking ROS 2...${NC}"
if [ -f "/opt/ros/humble/setup.bash" ]; then
    echo "✓ ROS 2 Humble is installed"
    source /opt/ros/humble/setup.bash
    if command -v ros2 &> /dev/null; then
        echo "✓ ROS 2 commands available"
    else
        echo -e "${RED}✗ ROS 2 commands not found${NC}"
    fi
else
    echo -e "${RED}✗ ROS 2 Humble not found${NC}"
fi

# Check disk space
echo -e "${GREEN}Checking disk space...${NC}"
DISK_USAGE=$(df -h / | awk 'NR==2 {print $5}' | sed 's/%//')
if [ "$DISK_USAGE" -lt 80 ]; then
    echo "✓ Disk usage: ${DISK_USAGE}%"
else
    echo -e "${YELLOW}⚠ Disk usage: ${DISK_USAGE}% (consider cleanup)${NC}"
fi

# Check memory
echo -e "${GREEN}Checking memory...${NC}"
MEM_INFO=$(free -h | awk 'NR==2{printf "%.0f", $3/$2 * 100}')
echo "Memory usage: ${MEM_INFO}%"

# Check CPU temperature (Jetson-specific)
echo -e "${GREEN}Checking CPU temperature...${NC}"
if [ -f "/sys/class/thermal/thermal_zone0/temp" ]; then
    TEMP=$(cat /sys/class/thermal/thermal_zone0/temp)
    TEMP_C=$((TEMP / 1000))
    if [ "$TEMP_C" -lt 70 ]; then
        echo "✓ CPU temperature: ${TEMP_C}°C"
    else
        echo -e "${YELLOW}⚠ CPU temperature: ${TEMP_C}°C (monitor closely)${NC}"
    fi
else
    echo "⚠ Temperature sensor not available"
fi

# Check network connectivity
echo -e "${GREEN}Checking network...${NC}"
if ping -c 1 -W 2 8.8.8.8 &> /dev/null; then
    echo "✓ Internet connectivity OK"
else
    echo -e "${YELLOW}⚠ No internet connectivity${NC}"
fi

# Check SSH
echo -e "${GREEN}Checking SSH...${NC}"
if systemctl is-active --quiet ssh; then
    echo "✓ SSH service is running"
else
    echo -e "${YELLOW}⚠ SSH service is not running${NC}"
fi

# Check mDNS
echo -e "${GREEN}Checking mDNS...${NC}"
if systemctl is-active --quiet avahi-daemon; then
    echo "✓ Avahi daemon is running"
else
    echo -e "${YELLOW}⚠ Avahi daemon is not running${NC}"
fi

# Check NVMe/SSD (if present)
echo -e "${GREEN}Checking NVMe SSD...${NC}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAAC_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
if [ -b /dev/nvme0n1 ]; then
    echo "✓ NVMe device present: /dev/nvme0n1"
    ROOT_DEV=$(findmnt -n -o SOURCE / 2>/dev/null | sed 's/p[0-9]*$//' || true)
    NVME_ABS=$(readlink -f /dev/nvme0n1 2>/dev/null || echo "")
    if [ -n "$ROOT_DEV" ] && [ -n "$NVME_ABS" ] && [ "$(readlink -f "$ROOT_DEV" 2>/dev/null)" = "$NVME_ABS" ]; then
        echo "✓ Booted from NVMe SSD (root on NVMe)"
    else
        echo "  Boot device: $(findmnt -n -o SOURCE / 2>/dev/null || echo 'unknown') (SD card - NVMe available for migration)"
    fi
    # SMART health (requires smartmontools, may need sudo)
    if command -v smartctl &>/dev/null; then
        if smartctl -H /dev/nvme0 2>/dev/null | grep -qE "overall-health|PASSED|OK"; then
            echo "✓ NVMe SMART health: OK"
        else
            echo -e "${YELLOW}⚠ NVMe SMART check skipped (run: sudo ./scripts/system/verify_ssd.sh for full verification)${NC}"
        fi
    else
        echo "  smartmontools not installed (apt install smartmontools for SMART checks)"
    fi
    # Capacity (lsblk works without root; blockdev may need sudo)
    if [ -b /dev/nvme0n1 ]; then
        SIZE_GB=$(lsblk -b -d -n -o SIZE /dev/nvme0n1 2>/dev/null | awk '{printf "%d", $1/1024/1024/1024}')
        if [ -n "$SIZE_GB" ] && [ "$SIZE_GB" -gt 0 ]; then
            echo "  NVMe capacity: ${SIZE_GB}GB"
        else
            echo "  NVMe capacity: (run with sudo for full details)"
        fi
    fi
    echo "  Full verification: sudo $ISAAC_ROOT/scripts/system/verify_ssd.sh"
else
    echo "  No NVMe device detected (optional - add NVMe for SSD migration)"
fi

echo ""
echo "=========================================="
echo "Health Check Complete"
echo "=========================================="

