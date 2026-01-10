#!/bin/bash
# Quick Deploy - Fast deployment of local code changes
# Uses rsync for speed, only syncs changed files

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

TARGET_HOST="${TARGET_HOST:-isaac.local}"
TARGET_USER="${TARGET_USER:-nano}"
TARGET_DEV_DIR="${TARGET_DEV_DIR:-/home/nano/src/jetson-orin-nano}"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo "=========================================="
echo "Quick Deploy - Fast Code Sync"
echo "=========================================="
echo "Target: ${TARGET_USER}@${TARGET_HOST}:${TARGET_DEV_DIR}"
echo ""

# Check SSH connection
if ! ssh -o ConnectTimeout=5 "${TARGET_USER}@${TARGET_HOST}" "echo 'SSH OK'" &> /dev/null; then
    echo "Error: Cannot connect to ${TARGET_USER}@${TARGET_HOST}"
    exit 1
fi

# Build ROS 2 packages locally if needed
echo -e "${BLUE}Building ROS 2 packages...${NC}"
cd "$PROJECT_ROOT"
if [ -d "~/ros2_ws/src" ]; then
    source /opt/ros/humble/setup.bash 2>/dev/null || true
    if [ -f ~/ros2_ws/install/setup.bash ]; then
        source ~/ros2_ws/install/setup.bash 2>/dev/null || true
    fi
    cd ~/ros2_ws
    colcon build --symlink-install 2>&1 | tail -5 || echo "Build completed"
fi

# Sync source code (exclude build artifacts)
echo -e "${GREEN}Syncing source code...${NC}"
rsync -avz --delete \
    --exclude='.git' \
    --exclude='__pycache__' \
    --exclude='*.pyc' \
    --exclude='.venv' \
    --exclude='venv' \
    --exclude='build' \
    --exclude='install' \
    --exclude='log' \
    --exclude='*.egg-info' \
    --exclude='dist' \
    --exclude='.setup_state' \
    --exclude='.setup.log' \
    --exclude='.ros' \
    --exclude='*.swp' \
    --exclude='*.swo' \
    --exclude='*~' \
    "${PROJECT_ROOT}/" "${TARGET_USER}@${TARGET_HOST}:${TARGET_DEV_DIR}/"

# Rebuild ROS 2 workspace on target
echo -e "${GREEN}Rebuilding ROS 2 workspace on target...${NC}"
ssh "${TARGET_USER}@${TARGET_HOST}" << EOF
    set -e
    source /opt/ros/humble/setup.bash 2>/dev/null || true
    cd ~/ros2_ws

    # Link packages if needed
    if [ ! -L src/hello_world ] && [ -d ${TARGET_DEV_DIR}/src/hello_world ]; then
        ln -sf ${TARGET_DEV_DIR}/src/hello_world src/ 2>/dev/null || true
    fi
    if [ ! -L src/system_monitor ] && [ -d ${TARGET_DEV_DIR}/src/system_monitor ]; then
        ln -sf ${TARGET_DEV_DIR}/src/system_monitor src/ 2>/dev/null || true
    fi
    if [ ! -L src/isaac_robot ] && [ -d ${TARGET_DEV_DIR}/src/isaac_robot ]; then
        ln -sf ${TARGET_DEV_DIR}/src/isaac_robot src/ 2>/dev/null || true
    fi

    # Build
    colcon build --symlink-install 2>&1 | tail -10 || echo "Build completed"
EOF

# Optionally restart services
echo ""
echo -e "${YELLOW}Restart services? (y/N)${NC}"
read -r response
if [[ "$response" =~ ^[Yy]$ ]]; then
    echo -e "${GREEN}Restarting services...${NC}"
    ssh "${TARGET_USER}@${TARGET_HOST}" << EOF
        systemctl --user restart isaac-robot.service 2>/dev/null || true
        sudo systemctl restart isaac-system-monitor.service 2>/dev/null || true
EOF
fi

echo ""
echo -e "${GREEN}=========================================="
echo "Quick Deploy Complete!"
echo "==========================================${NC}"
echo ""
echo "Code synced to: ${TARGET_DEV_DIR}"
echo "Changes are active immediately (dev takes precedence)"
echo ""
