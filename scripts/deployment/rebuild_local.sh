#!/bin/bash
# Rebuild Local - Quick rebuild of ROS 2 packages after code changes
# Faster than full deploy, just rebuilds packages locally

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m'

echo "=========================================="
echo "Rebuilding ROS 2 Packages"
echo "=========================================="
echo ""

# Source ROS 2
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
else
    echo "Error: ROS 2 Humble not found"
    exit 1
fi

# Source workspace if exists
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi

# Ensure workspace exists
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Link packages from dev sandbox
echo -e "${BLUE}Linking packages...${NC}"
if [ -d "${PROJECT_ROOT}/src/hello_world" ]; then
    ln -sf "${PROJECT_ROOT}/src/hello_world" src/ 2>/dev/null || true
fi
if [ -d "${PROJECT_ROOT}/src/system_monitor" ]; then
    ln -sf "${PROJECT_ROOT}/src/system_monitor" src/ 2>/dev/null || true
fi
if [ -d "${PROJECT_ROOT}/src/isaac_robot" ]; then
    ln -sf "${PROJECT_ROOT}/src/isaac_robot" src/ 2>/dev/null || true
fi

# Build packages
echo -e "${GREEN}Building packages...${NC}"
colcon build --symlink-install "$@" 2>&1 | tail -20

# Source new build
source install/setup.bash

echo ""
echo -e "${GREEN}=========================================="
echo "Rebuild Complete!"
echo "==========================================${NC}"
echo ""
echo "Packages rebuilt. Source workspace:"
echo "  source ~/ros2_ws/install/setup.bash"
echo ""
