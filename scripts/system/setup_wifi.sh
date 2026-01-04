#!/bin/bash
# Setup WiFi with Ethernet Priority
# Configures WiFi to work as fallback when Ethernet is disconnected
# Ethernet is preferred, WiFi falls back automatically

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m'

echo "=========================================="
echo "WiFi Setup - Ethernet Priority"
echo "=========================================="
echo ""
echo "This will configure WiFi to work as fallback when Ethernet is disconnected."
echo "Ethernet will be preferred when available."
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo -e "${RED}Error: This script must be run as root or with sudo${NC}"
    exit 1
fi

# Check for WiFi device
WIFI_DEVICE=$(nmcli device status | grep wifi | grep -v p2p | awk '{print $1}' | head -1)
if [ -z "$WIFI_DEVICE" ]; then
    echo -e "${RED}Error: No WiFi device found${NC}"
    exit 1
fi

echo -e "${BLUE}Found WiFi device: $WIFI_DEVICE${NC}"
echo ""

# Check for Ethernet device
ETH_DEVICE=$(nmcli device status | grep ethernet | grep -v docker | awk '{print $1}' | head -1)
if [ -z "$ETH_DEVICE" ]; then
    echo -e "${YELLOW}Warning: No Ethernet device found${NC}"
else
    echo -e "${BLUE}Found Ethernet device: $ETH_DEVICE${NC}"
fi
echo ""

# Set Ethernet priority (lower metric = higher priority)
if [ -n "$ETH_DEVICE" ]; then
    ETH_CONN=$(nmcli -t -f NAME,DEVICE connection show | grep "$ETH_DEVICE" | cut -d: -f1 | head -1)
    if [ -n "$ETH_CONN" ]; then
        echo -e "${GREEN}Configuring Ethernet connection priority...${NC}"
        # Set lower route metric for Ethernet (preferred)
        nmcli connection modify "$ETH_CONN" ipv4.route-metric 100 2>/dev/null || true
        nmcli connection modify "$ETH_CONN" ipv6.route-metric 100 2>/dev/null || true
        echo "✓ Ethernet priority set (metric: 100)"
    fi
fi

# Check if WiFi is already configured
WIFI_CONN=$(nmcli -t -f NAME,TYPE connection show | grep wifi | cut -d: -f1 | head -1)

if [ -n "$WIFI_CONN" ]; then
    echo ""
    echo -e "${BLUE}WiFi connection '$WIFI_CONN' already exists${NC}"
    echo "Update existing connection? (y/N)"
    read -r response
    if [[ ! "$response" =~ ^[Yy]$ ]]; then
        echo "Skipping WiFi configuration"
        exit 0
    fi

    # Update existing connection
    echo -e "${GREEN}Updating WiFi connection priority...${NC}"
    # Set higher route metric for WiFi (fallback)
    nmcli connection modify "$WIFI_CONN" ipv4.route-metric 200 2>/dev/null || true
    nmcli connection modify "$WIFI_CONN" ipv6.route-metric 200 2>/dev/null || true
    nmcli connection modify "$WIFI_CONN" connection.autoconnect yes 2>/dev/null || true
    echo "✓ WiFi priority set (metric: 200 - fallback)"
else
    # Scan for WiFi networks
    echo -e "${BLUE}Scanning for WiFi networks...${NC}"
    nmcli device wifi rescan 2>/dev/null || true
    sleep 2

    echo ""
    echo "Available WiFi networks:"
    nmcli device wifi list | head -20

    echo ""
    echo "Enter WiFi SSID (network name):"
    read -r SSID

    if [ -z "$SSID" ]; then
        echo -e "${YELLOW}No SSID provided, skipping WiFi setup${NC}"
        exit 0
    fi

    echo "Enter WiFi password (leave empty for open network):"
    read -r PASSWORD

    echo ""
    echo -e "${GREEN}Creating WiFi connection...${NC}"

    # Create WiFi connection
    if [ -z "$PASSWORD" ]; then
        nmcli connection add \
            type wifi \
            con-name "WiFi-$SSID" \
            ifname "$WIFI_DEVICE" \
            ssid "$SSID" \
            wifi-sec.key-mgmt none \
            ipv4.method auto \
            ipv4.route-metric 200 \
            ipv6.route-metric 200 \
            connection.autoconnect yes \
            connection.autoconnect-priority 10
    else
        nmcli connection add \
            type wifi \
            con-name "WiFi-$SSID" \
            ifname "$WIFI_DEVICE" \
            ssid "$SSID" \
            wifi-sec.key-mgmt wpa-psk \
            wifi-sec.psk "$PASSWORD" \
            ipv4.method auto \
            ipv4.route-metric 200 \
            ipv6.route-metric 200 \
            connection.autoconnect yes \
            connection.autoconnect-priority 10
    fi

    echo "✓ WiFi connection created"
fi

# Configure NetworkManager to prefer Ethernet
echo ""
echo -e "${GREEN}Configuring NetworkManager preferences...${NC}"

# Ensure NetworkManager manages all devices
nmcli device set "$WIFI_DEVICE" managed yes 2>/dev/null || true

# Set connection priority in NetworkManager config
if ! grep -q "^\[connection\]" /etc/NetworkManager/NetworkManager.conf; then
    echo "" >> /etc/NetworkManager/NetworkManager.conf
    echo "[connection]" >> /etc/NetworkManager/NetworkManager.conf
fi

# Restart NetworkManager to apply changes
systemctl restart NetworkManager
sleep 2

echo "✓ NetworkManager configured"
echo ""

# Summary
echo -e "${GREEN}=========================================="
echo "WiFi Setup Complete!"
echo "==========================================${NC}"
echo ""
echo "Configuration:"
echo "  - Ethernet priority: 100 (preferred)"
echo "  - WiFi priority: 200 (fallback)"
echo "  - WiFi will auto-connect when Ethernet is disconnected"
echo ""
echo "To test:"
echo "  1. Disconnect Ethernet cable"
echo "  2. WiFi should connect automatically"
echo "  3. Reconnect Ethernet - it will take priority"
echo ""
echo "Check status:"
echo "  nmcli device status"
echo "  nmcli connection show"
echo ""
