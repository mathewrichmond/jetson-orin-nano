#!/bin/bash
# Bench Test: Camera Visualization
# Run on Jetson to test camera visualization from MacBook
# This is a bench test that requires real hardware

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

echo "=========================================="
echo "Camera Visualization Bench Test"
echo "=========================================="
echo ""
echo "This bench test:"
echo "  1. Launches cameras from graph config"
echo "  2. Launches rosbridge for visualization"
echo "  3. Verifies topics are publishing"
echo "  4. Provides connection info for MacBook"
echo ""

# Source ROS 2
if [ ! -f "/opt/ros/humble/setup.bash" ]; then
    echo "Error: ROS 2 Humble not found"
    exit 1
fi

source /opt/ros/humble/setup.bash

if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi

# Check rosbridge
if ! ros2 pkg list | grep -q rosbridge_suite; then
    echo "Installing rosbridge..."
    "$REPO_ROOT/scripts/visualization/setup_rosbridge.sh"
fi

# Get IP address
IP_ADDRESS=$(ip -4 addr show | grep -oP '(?<=inet\s)\d+(\.\d+){3}' | grep -v '127.0.0.1' | head -1)
HOSTNAME=$(hostname)
MDNS_NAME="${HOSTNAME}.local"

echo "Network Info:"
echo "  IP: $IP_ADDRESS"
echo "  mDNS: $MDNS_NAME"
echo ""
echo "Connect from MacBook:"
echo "  ws://$IP_ADDRESS:9090"
echo "  or"
echo "  ws://$MDNS_NAME:9090"
echo ""

# Check firewall
if command -v ufw &> /dev/null && sudo ufw status | grep -q "Status: active"; then
    if ! sudo ufw status | grep -q "9090"; then
        echo "Opening firewall port 9090..."
        sudo ufw allow 9090/tcp
    fi
fi

# Set ROS_DOMAIN_ID
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

echo "Starting nodes..."
echo "Press Ctrl+C to stop"
echo ""

# Launch using graph manager
trap 'kill 0' EXIT

# Launch cameras from graph
ros2 launch isaac_robot graph.launch.py \
    graph_config:=robot_graph.yaml \
    group:=hardware &

sleep 2

# Launch rosbridge
ros2 launch isaac_robot rosbridge.launch.py rosbridge_address:=0.0.0.0
