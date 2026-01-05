#!/bin/bash
# Start Visualization Services
# Launches visualization tools (rosbridge or Foxglove Bridge) for remote visualization

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UTILS_DIR="${SCRIPT_DIR}/../utils"

# Find Isaac root (dev takes precedence)
ISAAC_ROOT="${ISAAC_ROOT:-$(python3 "${UTILS_DIR}/find_isaac_root.py" 2>/dev/null || echo "/opt/isaac-robot")}"

# Visualization backend selection (environment variable > default)
VIZ_BACKEND="${VIZ_BACKEND:-foxglove}"  # Options: rosbridge, foxglove

# Get connection parameters
VIZ_ADDRESS="${VIZ_ADDRESS:-0.0.0.0}"
ROSBRIDGE_PORT="${ROSBRIDGE_PORT:-9090}"
FOXGLOVE_PORT="${FOXGLOVE_PORT:-8765}"

echo "=========================================="
echo "Starting Visualization Services"
echo "=========================================="
echo "Isaac Root: $ISAAC_ROOT"
echo "Backend: $VIZ_BACKEND"
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

# Launch visualization backend
echo "Launching visualization backend: $VIZ_BACKEND"
case "$VIZ_BACKEND" in
    rosbridge)
        echo "Starting rosbridge on port $ROSBRIDGE_PORT..."
        ros2 launch isaac_robot rosbridge.launch.py \
            rosbridge_address:="$VIZ_ADDRESS" \
            rosbridge_port:="$ROSBRIDGE_PORT"
        ;;
    foxglove)
        echo "Starting Foxglove Bridge on port $FOXGLOVE_PORT..."
        ros2 launch isaac_robot foxglove_bridge.launch.py \
            address:="$VIZ_ADDRESS" \
            port:="$FOXGLOVE_PORT"
        ;;
    *)
        echo "Error: Unknown visualization backend '$VIZ_BACKEND'"
        echo "Supported backends: rosbridge, foxglove"
        exit 1
        ;;
esac
