#!/bin/bash
# Test Hello World Node

set -e

echo "=========================================="
echo "Testing Hello World Node"
echo "=========================================="

# Source ROS 2
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash 2>/dev/null || {
    echo "Building workspace..."
    cd ~/ros2_ws
    colcon build --packages-select hello_world
    source install/setup.bash
}

echo ""
echo "Starting hello_world node..."
echo "Press Ctrl+C to stop"
echo ""

# Run node
ros2 run hello_world hello_world_node
