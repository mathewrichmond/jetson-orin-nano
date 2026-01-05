#!/bin/bash
# Bench Test Script: Visualize RealSense cameras from MacBook
# Run this on the Jetson (target) to set up camera + rosbridge

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

echo "=========================================="
echo "RealSense Camera Bench Test Setup"
echo "=========================================="
echo ""
echo "This script will:"
echo "  1. Check rosbridge installation"
echo "  2. Get Jetson IP address"
echo "  3. Launch cameras + rosbridge"
echo ""
echo "Then connect from your MacBook using Foxglove Studio"
echo ""

# Check if ROS 2 is installed
if [ ! -f "/opt/ros/humble/setup.bash" ]; then
    echo "Error: ROS 2 Humble not found. Please install ROS 2 first."
    exit 1
fi

source /opt/ros/humble/setup.bash

# Check if workspace exists and source it
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi

# Check rosbridge installation
echo "Checking rosbridge installation..."
if ! ros2 pkg list | grep -q rosbridge_suite; then
    echo "⚠️  rosbridge_suite not found. Installing..."
    "$SCRIPT_DIR/setup_rosbridge.sh"
else
    echo "✓ rosbridge_suite found"
fi

# Get IP address
echo ""
echo "=========================================="
echo "Network Configuration"
echo "=========================================="

# Try to get IP from various interfaces
IP_ADDRESS=""
if command -v hostname &> /dev/null; then
    HOSTNAME=$(hostname)
    echo "Hostname: $HOSTNAME"
fi

# Get IP address
if command -v ip &> /dev/null; then
    IP_ADDRESS=$(ip -4 addr show | grep -oP '(?<=inet\s)\d+(\.\d+){3}' | grep -v '127.0.0.1' | head -1)
elif command -v ifconfig &> /dev/null; then
    IP_ADDRESS=$(ifconfig | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1' | head -1)
fi

if [ -n "$IP_ADDRESS" ]; then
    echo "IP Address: $IP_ADDRESS"
else
    echo "⚠️  Could not determine IP address automatically"
    echo "Please find your IP address manually:"
    echo "  ip addr show"
    echo "  or"
    echo "  ifconfig"
    IP_ADDRESS="<JETSON_IP>"
fi

# Check if mDNS is available
if command -v avahi-resolve &> /dev/null; then
    MDNS_NAME=$(hostname).local
    echo "mDNS name: $MDNS_NAME"
    echo ""
    echo "You can connect using either:"
    echo "  - IP: $IP_ADDRESS"
    echo "  - mDNS: $MDNS_NAME"
else
    echo ""
    echo "Connect using IP: $IP_ADDRESS"
fi

echo ""
echo "=========================================="
echo "Connection Information for MacBook"
echo "=========================================="
echo ""
echo "Rosbridge WebSocket URL:"
echo "  ws://$IP_ADDRESS:9090"
echo ""
if [ -n "$MDNS_NAME" ]; then
    echo "Or using mDNS:"
    echo "  ws://$MDNS_NAME:9090"
    echo ""
fi
echo "=========================================="
echo ""

# Check firewall
echo "Checking firewall..."
if command -v ufw &> /dev/null; then
    if sudo ufw status | grep -q "Status: active"; then
        if ! sudo ufw status | grep -q "9090"; then
            echo "⚠️  Firewall is active but port 9090 may not be open"
            echo "   Run: sudo ufw allow 9090/tcp"
            read -p "   Allow port 9090 now? (y/n) " -n 1 -r
            echo
            if [[ $REPLY =~ ^[Yy]$ ]]; then
                sudo ufw allow 9090/tcp
                echo "✓ Port 9090 opened"
            fi
        else
            echo "✓ Port 9090 is open"
        fi
    else
        echo "✓ Firewall is not active"
    fi
fi

echo ""
echo "=========================================="
echo "Starting Camera Node + Rosbridge"
echo "=========================================="
echo ""
echo "Press Ctrl+C to stop"
echo ""

# Set ROS_DOMAIN_ID (default to 0)
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo ""

# Launch both camera and rosbridge in the same process group
# This allows Ctrl+C to stop both
trap 'kill 0' EXIT

# Launch camera node + rosbridge using graph launcher
echo "Starting camera node + rosbridge..."
ros2 launch isaac_robot graph.launch.py \
    graph_config:=robot_graph.yaml \
    group:=hardware &

# Launch rosbridge separately (not in graph yet)
sleep 2
echo "Starting rosbridge server..."
ros2 launch isaac_robot rosbridge.launch.py rosbridge_address:=0.0.0.0
