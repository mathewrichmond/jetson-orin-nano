#!/bin/bash
# Safe ROS 2 Workspace Sourcing Script
# Properly sources ROS 2 workspace without path conflicts
# Usage: source scripts/utils/source_ros2.sh

# Unset any incorrectly set prefix variables
unset AMENT_CURRENT_PREFIX COLCON_CURRENT_PREFIX

# Source ROS 2 base installation
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

# Source ROS 2 workspace
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi

echo "✓ ROS 2 workspace sourced successfully"
