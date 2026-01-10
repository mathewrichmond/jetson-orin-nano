#!/bin/bash
# Check if pointcloud topics are available
# Run this from your MacBook to verify pointcloud topics are publishing

JETSON_IP=${1:-192.168.0.129}

echo "Checking for pointcloud topics on Jetson ($JETSON_IP)..."
echo ""

# Check if we can reach the Jetson
if ! ping -c 1 -W 2 "$JETSON_IP" > /dev/null 2>&1; then
    echo "Error: Cannot reach Jetson at $JETSON_IP"
    echo "Make sure:"
    echo "  1. Jetson is powered on"
    echo "  2. Both computers are on the same network"
    exit 1
fi

echo "To check pointcloud topics, run these commands ON THE JETSON:"
echo ""
echo "1. List all camera topics:"
echo "   ros2 topic list | grep camera"
echo ""
echo "2. Check if pointcloud topics exist:"
echo "   ros2 topic list | grep points"
echo ""
echo "3. Check pointcloud publishing rate:"
echo "   ros2 topic hz /camera_front/points"
echo "   ros2 topic hz /camera_rear/points"
echo ""
echo "4. If pointclouds aren't publishing, enable them:"
echo "   # Edit config/robot/robot_graph.yaml"
echo "   # Set enable_pointcloud: true"
echo "   # Then restart camera node"
echo ""
echo "5. Verify pointcloud is enabled in config:"
echo "   grep enable_pointcloud config/robot/robot_graph.yaml"
echo ""
