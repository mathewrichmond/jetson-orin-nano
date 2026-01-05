#!/bin/bash
# Setup script for rosbridge visualization
# Installs rosbridge_suite for web-based visualization

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

echo "=========================================="
echo "Setting up rosbridge for visualization"
echo "=========================================="

# Check if ROS 2 is installed
if [ ! -f "/opt/ros/humble/setup.bash" ]; then
    echo "Error: ROS 2 Humble not found. Please install ROS 2 first."
    exit 1
fi

source /opt/ros/humble/setup.bash

# Check if workspace exists
if [ ! -d ~/ros2_ws/src ]; then
    echo "Creating ROS 2 workspace..."
    mkdir -p ~/ros2_ws/src
fi

cd ~/ros2_ws

# Check if rosbridge_suite is already installed via apt
if dpkg -l | grep -q ros-humble-rosbridge-suite; then
    echo "rosbridge_suite already installed via apt"
else
    echo "Installing rosbridge_suite..."

    # Try apt install first
    if sudo apt install -y ros-humble-rosbridge-suite 2>/dev/null; then
        echo "✓ Installed rosbridge_suite via apt"
    else
        echo "Building rosbridge_suite from source..."

        # Clone rosbridge_suite
        if [ ! -d src/rosbridge_suite ]; then
            cd src
            git clone https://github.com/RobotWebTools/rosbridge_suite.git -b humble
            cd ..
        fi

        # Install dependencies
        if command -v rosdep &> /dev/null; then
            rosdep update || true
            rosdep install --from-paths src --ignore-src -r -y || true
        fi

        # Build
        colcon build --packages-select rosbridge_suite

        echo "✓ Built rosbridge_suite from source"
    fi
fi

# Source workspace
source install/setup.bash

echo ""
echo "=========================================="
echo "rosbridge setup complete!"
echo "=========================================="
echo ""
echo "Usage:"
echo "  ros2 launch isaac_robot rosbridge.launch.py"
echo ""
echo "Then connect via:"
echo "  - Foxglove Studio: ws://<robot-ip>:9090"
echo "  - Web browser: studio.foxglove.dev"
echo ""
echo "Note: Visualization works with any ROS 2 topics, not just cameras."
echo ""
