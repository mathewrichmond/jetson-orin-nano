#!/bin/bash
# Launch Full Modular System with All Hardware
# This launches all hardware groups

set -e

cd /home/nano/src/jetson-orin-nano
source install/setup.bash

echo "========================================="
echo "Launching Full Modular Robot System"
echo "========================================="
echo ""
echo "Hardware Groups:"
echo "  ✓ Core (system + health monitoring)"
echo "  ✓ Vision Pipeline (2x RealSense D435)"
echo "  ✓ Audio Pipeline (USB Microphone)"
echo "  ✓ Chassis Control (IMU + Motors)"
echo "  ✓ Power Management"
echo ""
echo "Press Ctrl+C to stop"
echo ""

# Launch with 'all' group to start everything
exec ros2 launch isaac_robot graph.launch.py \
  graph_config:=modular_graph.yaml \
  group:=all
