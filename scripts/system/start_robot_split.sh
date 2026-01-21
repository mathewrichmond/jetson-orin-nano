#!/bin/bash
# Start Robot System with Split Launch
# Launches camera pipeline in composable container, other nodes separately

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UTILS_DIR="${SCRIPT_DIR}/../utils"

# Find Isaac root
ISAAC_ROOT="${ISAAC_ROOT:-$(python3 "${UTILS_DIR}/find_isaac_root.py" 2>/dev/null || echo "/opt/isaac-robot")}"

echo "=========================================="
echo "Starting Isaac Robot System (Split Mode)"
echo "=========================================="
echo "Isaac Root: $ISAAC_ROOT"
echo ""

# Source ROS 2
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi

# Configure FastDDS
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
if [ -z "$RMW_IMPLEMENTATION" ]; then
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
fi
if [ -f "$PROJECT_ROOT/config/ros2/fastdds_profiles.xml" ]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="$PROJECT_ROOT/config/ros2/fastdds_profiles.xml"
    export RMW_FASTRTPS_USE_QOS_FROM_XML=1
fi
export RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS

cd "$ISAAC_ROOT"

echo "Launching camera pipeline in composable container (background)..."
ros2 launch isaac_robot composable_graph.launch.py graph_config:=robot_graph.yaml group:=composable_pipeline use_composable:=true &
PIPELINE_PID=$!

sleep 3

echo "Launching other nodes..."
ros2 launch isaac_robot composable_graph.launch.py graph_config:=robot_graph.yaml group:=all use_composable:=false &
OTHER_PID=$!

# Wait for both processes
wait $PIPELINE_PID $OTHER_PID
