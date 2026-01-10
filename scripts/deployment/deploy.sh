#!/bin/bash
# Deployment Script
# Deploys code to the Jetson Orin Nano system

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

# Default values
TARGET_HOST="${TARGET_HOST:-isaac.local}"
TARGET_USER="${TARGET_USER:-nano}"
TARGET_DIR="${TARGET_DIR:-~/src/jetson-orin-nano}"
BUILD_TYPE="${BUILD_TYPE:-install}"

usage() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  -h, --host HOST       Target host (default: isaac.local)"
    echo "  -u, --user USER       Target user (default: nano)"
    echo "  -d, --dir DIR         Target directory (default: ~/src/jetson-orin-nano)"
    echo "  -b, --build TYPE      Build type: install|build|test (default: install)"
    echo "  --skip-build          Skip building before deployment"
    echo "  --skip-ros            Skip ROS 2 workspace build"
    echo "  -v, --verbose         Verbose output"
    echo "  --help                Show this help message"
    exit 1
}

# Parse arguments
SKIP_BUILD=false
SKIP_ROS=false
VERBOSE=false

while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--host)
            TARGET_HOST="$2"
            shift 2
            ;;
        -u|--user)
            TARGET_USER="$2"
            shift 2
            ;;
        -d|--dir)
            TARGET_DIR="$2"
            shift 2
            ;;
        -b|--build)
            BUILD_TYPE="$2"
            shift 2
            ;;
        --skip-build)
            SKIP_BUILD=true
            shift
            ;;
        --skip-ros)
            SKIP_ROS=true
            shift
            ;;
        -v|--verbose)
            VERBOSE=true
            shift
            ;;
        --help)
            usage
            ;;
        *)
            echo "Unknown option: $1"
            usage
            ;;
    esac
done

echo "=========================================="
echo "Deploying to Isaac Robot System"
echo "=========================================="
echo "Target: ${TARGET_USER}@${TARGET_HOST}:${TARGET_DIR}"
echo ""

# Check SSH connection
echo -e "${GREEN}Checking SSH connection...${NC}"
if ! ssh -o ConnectTimeout=5 "${TARGET_USER}@${TARGET_HOST}" "echo 'SSH connection OK'" &> /dev/null; then
    echo -e "${RED}Error: Cannot connect to ${TARGET_USER}@${TARGET_HOST}${NC}"
    echo "Please ensure:"
    echo "  1. SSH is enabled on the target"
    echo "  2. SSH keys are set up"
    echo "  3. Hostname resolves correctly"
    exit 1
fi

# Build locally if requested
if [ "$SKIP_BUILD" = false ]; then
    echo -e "${GREEN}Building project...${NC}"
    cd "$PROJECT_ROOT"

    # Run pre-commit checks
    if command -v pre-commit &> /dev/null; then
        echo "Running pre-commit checks..."
        pre-commit run --all-files || echo "Pre-commit checks failed, continuing..."
    fi
fi

# Create deployment archive
echo -e "${GREEN}Creating deployment archive...${NC}"
TEMP_DIR=$(mktemp -d)
ARCHIVE="${TEMP_DIR}/isaac-deploy.tar.gz"

cd "$PROJECT_ROOT"
tar --exclude='.git' \
    --exclude='__pycache__' \
    --exclude='*.pyc' \
    --exclude='.venv' \
    --exclude='venv' \
    --exclude='build' \
    --exclude='install' \
    --exclude='log' \
    --exclude='*.egg-info' \
    -czf "$ARCHIVE" .

# Deploy to target
echo -e "${GREEN}Deploying to ${TARGET_HOST}...${NC}"
scp "$ARCHIVE" "${TARGET_USER}@${TARGET_HOST}:/tmp/isaac-deploy.tar.gz"

# Extract on target
echo -e "${GREEN}Extracting on target...${NC}"
ssh "${TARGET_USER}@${TARGET_HOST}" << EOF
    set -e
    mkdir -p ${TARGET_DIR}
    cd ${TARGET_DIR}
    tar -xzf /tmp/isaac-deploy.tar.gz
    rm /tmp/isaac-deploy.tar.gz
EOF

# Build ROS 2 workspace on target if requested
if [ "$SKIP_ROS" = false ]; then
    echo -e "${GREEN}Building ROS 2 workspace on target...${NC}"
    ssh "${TARGET_USER}@${TARGET_HOST}" << EOF
        set -e
        source /opt/ros/humble/setup.bash 2>/dev/null || true
        cd ~/ros2_ws/src
        # Link or copy system_monitor if it exists
        if [ -d ${TARGET_DIR}/src/system_monitor ]; then
            ln -sf ${TARGET_DIR}/src/system_monitor system_monitor 2>/dev/null || true
        fi
        cd ~/ros2_ws
        colcon build --symlink-install || true
EOF
fi

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

# Cleanup
rm -rf "$TEMP_DIR"

echo ""
echo -e "${GREEN}=========================================="
echo "Deployment Complete!"
echo "==========================================${NC}"
echo ""
echo "Next steps on target:"
echo "  ssh ${TARGET_USER}@${TARGET_HOST}"
echo "  source ~/ros2_ws/install/setup.bash"
echo "  ros2 launch system_monitor system_monitor.launch.py"
