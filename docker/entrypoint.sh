#!/bin/bash
set -e

# Source ROS 2
source /opt/ros/humble/setup.sh

# Source workspace if built
if [ -f /workspace/install/setup.sh ]; then
    source /workspace/install/setup.sh
fi

# Execute command
exec "$@"
