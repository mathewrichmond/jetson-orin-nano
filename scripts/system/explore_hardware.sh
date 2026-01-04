#!/bin/bash
# Hardware Exploration Script
# Documents current hardware configuration and identifies missing components

set -e

OUTPUT_FILE="${1:-hardware_report.txt}"

echo "=========================================="
echo "Isaac Hardware Exploration Report"
echo "Generated: $(date)"
echo "=========================================="
echo ""

# CPU Information
echo "=== CPU Information ==="
lscpu | grep -E "Architecture|CPU\(s\)|Model name|CPU max MHz|CPU min MHz"
echo ""

# Memory Information
echo "=== Memory Information ==="
free -h
echo ""

# Thermal Zones
echo "=== Thermal Zones ==="
for zone in /sys/class/thermal/thermal_zone*/type; do
    if [ -f "$zone" ]; then
        zone_num=$(echo "$zone" | grep -o '[0-9]*')
        temp_file="/sys/class/thermal/thermal_zone${zone_num}/temp"
        if [ -f "$temp_file" ]; then
            temp=$(cat "$temp_file")
            temp_c=$((temp / 1000))
            zone_name=$(cat "$zone")
            echo "Zone ${zone_num}: ${zone_name} - ${temp_c}°C"
        fi
    fi
done
echo ""

# USB Devices
echo "=== USB Devices ==="
lsusb
echo ""

# PCI Devices
echo "=== PCI Devices ==="
lspci
echo ""

# Network Interfaces
echo "=== Network Interfaces ==="
nmcli device status
echo ""
ip -br addr show
echo ""

# Storage
echo "=== Storage Information ==="
df -h
echo ""
lsblk
echo ""

# Jetson-Specific Information
echo "=== Jetson-Specific Information ==="
if command -v nvpmodel &> /dev/null; then
    echo "Power Mode:"
    sudo nvpmodel -q 2>/dev/null || echo "  Unable to query (may need sudo)"
fi
echo ""

if command -v tegrastats &> /dev/null; then
    echo "Tegrastats available: Yes"
else
    echo "Tegrastats available: No"
fi
echo ""

# Display/Graphics
echo "=== Display/Graphics ==="
if command -v xrandr &> /dev/null; then
    xrandr --listmonitors 2>/dev/null || echo "  No X server running"
else
    echo "xrandr not installed"
fi
echo ""

# Installed Display Packages
echo "=== Display-Related Packages ==="
dpkg -l | grep -E "xserver|wayland|desktop|display" | grep -v "^rc" | head -10
echo ""

# Power Consumption (if available)
echo "=== Power Information ==="
if [ -f "/sys/bus/i2c/drivers/ina3221x/1-0040/iio:device0/in_power0_input" ]; then
    echo "Power sensors found:"
    for sensor in /sys/bus/i2c/drivers/ina3221x/*/iio:device*/in_power*_input; do
        if [ -f "$sensor" ]; then
            power=$(cat "$sensor")
            echo "  $(basename $(dirname $(dirname $sensor))): ${power} mW"
        fi
    done
else
    echo "Power sensors not accessible (may need root)"
fi
echo ""

# Missing Packages Check
echo "=== Recommended Packages Check ==="
echo "Checking for common robot system packages..."
MISSING_PKGS=()

# Display packages
if ! dpkg -l | grep -q "xserver-xorg-core"; then
    MISSING_PKGS+=("xserver-xorg-core xserver-xorg-input-all")
fi

# WiFi tools
if ! command -v iwconfig &> /dev/null; then
    MISSING_PKGS+=("wireless-tools")
fi

# Additional monitoring tools
if ! command -v sensors &> /dev/null; then
    MISSING_PKGS+=("lm-sensors")
fi

if ! command -v iotop &> /dev/null; then
    MISSING_PKGS+=("iotop")
fi

if [ ${#MISSING_PKGS[@]} -eq 0 ]; then
    echo "All recommended packages appear to be installed"
else
    echo "Consider installing:"
    for pkg in "${MISSING_PKGS[@]}"; do
        echo "  - $pkg"
    done
fi
echo ""

echo "=========================================="
echo "Report complete"
echo "=========================================="

if [ -n "$OUTPUT_FILE" ] && [ "$OUTPUT_FILE" != "-" ]; then
    echo "Report saved to: $OUTPUT_FILE"
fi

