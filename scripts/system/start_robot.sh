#!/bin/bash
# Start Robot System
# Launches the robot system (dev or installed) with selected graph configuration

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UTILS_DIR="${SCRIPT_DIR}/../utils"

# Find Isaac root (dev takes precedence)
ISAAC_ROOT="${ISAAC_ROOT:-$(python3 "${UTILS_DIR}/find_isaac_root.py" 2>/dev/null || echo "/opt/isaac-robot")}"

# Get graph selection (environment variable > config file > default)
GRAPH_SELECTION="${ROBOT_GRAPH:-$("${UTILS_DIR}/get_graph.sh" 2>/dev/null || echo "minimal")}"

echo "=========================================="
echo "Starting Isaac Robot System"
echo "=========================================="
echo "Isaac Root: $ISAAC_ROOT"
echo "Graph Selection: $GRAPH_SELECTION"
echo ""

# Source ROS 2 if available
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

# Source ROS 2 workspace if available
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi

# Change to Isaac root
cd "$ISAAC_ROOT"

# Check if this is dev or installed
if [ "$ISAAC_ROOT" = "/home/nano/src/jetson-orin-nano" ]; then
    echo "Running from DEV sandbox"
    DEV_MODE=true
else
    echo "Running from INSTALLED package"
    DEV_MODE=false
fi

# Launch robot system with selected graph
echo "Launching robot system with graph: $GRAPH_SELECTION"
case "$GRAPH_SELECTION" in
    minimal)
        ros2 launch isaac_robot minimal.launch.py
        ;;
    full)
        ros2 launch isaac_robot full.launch.py
        ;;
    robot)
        ros2 launch isaac_robot robot.launch.py
        ;;
    *)
        echo "Warning: Unknown graph '$GRAPH_SELECTION', using minimal"
        ros2 launch isaac_robot minimal.launch.py
        ;;
esac
