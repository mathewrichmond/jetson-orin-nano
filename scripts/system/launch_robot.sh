#!/bin/bash
# Simple script to launch the robot graph
# Standard ROS 2 way: uses ros2 launch with graph configuration

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Source ROS 2
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

# Source workspace
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi

# Default arguments
GRAPH_CONFIG="${1:-robot_graph.yaml}"
GROUP="${2:-bench_test}"

# Find config file
CONFIG_PATH=""
if [ -f "$PROJECT_ROOT/config/robot/$GRAPH_CONFIG" ]; then
    CONFIG_PATH="$PROJECT_ROOT/config/robot/$GRAPH_CONFIG"
elif [ -f "$GRAPH_CONFIG" ]; then
    CONFIG_PATH="$GRAPH_CONFIG"
else
    echo "Error: Graph config file not found: $GRAPH_CONFIG"
    exit 1
fi

echo "=========================================="
echo "Launching Isaac Robot Graph"
echo "=========================================="
echo "Config: $CONFIG_PATH"
echo "Group: $GROUP"
echo "=========================================="
echo ""

# Launch using standard ROS 2 launch system
cd "$PROJECT_ROOT"
ros2 launch isaac_robot graph.launch.py \
    graph_config:="$CONFIG_PATH" \
    group:="$GROUP"
