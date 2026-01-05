#!/bin/bash
# Setup Foxglove Bridge for Foxglove Studio
# Alternative to rosbridge with better Foxglove Studio integration

set -e

echo "=========================================="
echo "Foxglove Bridge Setup"
echo "=========================================="
echo ""

# Check ROS 2
if [ ! -f "/opt/ros/humble/setup.bash" ]; then
    echo "Error: ROS 2 Humble not found"
    exit 1
fi

source /opt/ros/humble/setup.bash

# Check if already installed
if ros2 pkg list | grep -q foxglove_bridge; then
    echo "✓ Foxglove Bridge already installed"
else
    echo "Installing Foxglove Bridge..."
    sudo apt update
    sudo apt install -y ros-humble-foxglove-bridge

    if ! ros2 pkg list | grep -q foxglove_bridge; then
        echo "Error: Installation failed"
        exit 1
    fi
    echo "✓ Foxglove Bridge installed"
fi

echo ""
echo "=========================================="
echo "Usage"
echo "=========================================="
echo ""
echo "Launch Foxglove Bridge:"
echo "  ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765"
echo ""
echo "Or use the launch file:"
echo "  ros2 launch isaac_robot foxglove_bridge.launch.py"
echo ""
# Get IP address for connection info
IP_ADDRESS="<robot-ip>"
if command -v ip &> /dev/null; then
    IP_ADDRESS=$(ip -4 addr show 2>/dev/null | grep -oE 'inet [0-9]+\.[0-9]+\.[0-9]+\.[0-9]+' | grep -v '127.0.0.1' | awk '{print $2}' | head -1)
elif command -v hostname &> /dev/null; then
    IP_ADDRESS=$(hostname -I 2>/dev/null | awk '{print $1}' || echo "<robot-ip>")
fi

HOSTNAME=$(hostname 2>/dev/null || echo "isaac")
MDNS_NAME="${HOSTNAME}.local"

echo "In Foxglove Studio:"
echo "  1. Select 'ROS 2' framework"
echo "  2. Enter robot IP: $IP_ADDRESS (or $MDNS_NAME if mDNS configured)"
echo "  3. Port: 8765"
echo "  4. Connect"
echo ""
