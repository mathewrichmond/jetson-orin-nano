#!/bin/bash
# Start Robot System
# Launches the robot system (dev or installed)

set -e

# Find Isaac root (dev takes precedence)
ISAAC_ROOT="${ISAAC_ROOT:-$(python3 "$(dirname "$0")/../../scripts/utils/find_isaac_root.py" 2>/dev/null || echo "/opt/isaac-robot")}"

echo "=========================================="
echo "Starting Isaac Robot System"
echo "=========================================="
echo "Isaac Root: $ISAAC_ROOT"
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

# Launch system monitor (if not already running via systemd)
if ! systemctl is-active --quiet isaac-system-monitor.service; then
    echo "Launching system monitor..."
    ros2 launch system_monitor system_monitor.launch.py &
    sleep 2
fi

# Launch other components as they are added
# ros2 launch vla_controller vla_controller.launch.py &
# ros2 launch hardware_drivers hardware_drivers.launch.py &

# Keep running and monitor health
while true; do
    # Check if monitor is still running
    if ! systemctl is-active --quiet isaac-system-monitor.service && ! pgrep -f "system_monitor" > /dev/null; then
        echo "System monitor stopped, restarting..."
        ros2 launch system_monitor system_monitor.launch.py &
        sleep 2
    fi

    # Sleep and check again
    sleep 30
done
