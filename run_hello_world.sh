#!/bin/bash
# Run Hello World Example

set -e

echo "=========================================="
echo "Isaac Robot System - Hello World Test"
echo "=========================================="
echo ""

# Source ROS 2
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash 2>/dev/null || {
    echo "Building hello_world package..."
    cd ~/ros2_ws
    colcon build --packages-select hello_world
    source install/setup.bash
}

echo "Starting hello_world node..."
echo "This will publish messages every 2 seconds"
echo "Press Ctrl+C to stop"
echo ""

# Run node directly (entry point may not be registered yet)
python3 ~/ros2_ws/install/hello_world/local/lib/python3.10/dist-packages/hello_world/hello_world_node.py
