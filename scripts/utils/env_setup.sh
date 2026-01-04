#!/bin/bash
# Environment Setup Script
# Activates the development environment
# Can be sourced: source scripts/utils/env_setup.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Activate virtual environment if it exists
if [ -d "$PROJECT_ROOT/.venv" ]; then
    source "$PROJECT_ROOT/.venv/bin/activate"
fi

# Source ROS 2 if available
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

# Source ROS 2 workspace if it exists
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi

# Add local bin to PATH
export PATH="$HOME/.local/bin:$PATH"

# Set project root
export ISAAC_PROJECT_ROOT="$PROJECT_ROOT"
export PYTHONPATH="$PROJECT_ROOT:$PYTHONPATH"

# Change to project directory
cd "$PROJECT_ROOT"

# Print environment info
echo "Isaac Development Environment Activated"
echo "Project root: $PROJECT_ROOT"
echo "Python: $(which python3)"
if [ -f "/opt/ros/humble/setup.bash" ]; then
    echo "ROS 2: Available"
else
    echo "ROS 2: Not available"
fi

