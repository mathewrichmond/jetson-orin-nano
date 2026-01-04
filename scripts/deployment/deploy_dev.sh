#!/bin/bash
# Deploy Dev Changes
# Quickly deploy local changes to dev sandbox on target

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

TARGET_HOST="${TARGET_HOST:-isaac.local}"
TARGET_USER="${TARGET_USER:-nano}"
TARGET_DEV_DIR="${TARGET_DEV_DIR:-/home/nano/src/jetson-orin-nano}"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo "=========================================="
echo "Deploying Dev Changes to Target"
echo "=========================================="
echo "Target: ${TARGET_USER}@${TARGET_HOST}:${TARGET_DEV_DIR}"
echo ""

# Check SSH connection
if ! ssh -o ConnectTimeout=5 "${TARGET_USER}@${TARGET_HOST}" "echo 'SSH OK'" &> /dev/null; then
    echo "Error: Cannot connect to ${TARGET_USER}@${TARGET_HOST}"
    exit 1
fi

# Create deployment archive (exclude build artifacts)
echo -e "${GREEN}Creating deployment archive...${NC}"
TEMP_DIR=$(mktemp -d)
ARCHIVE="${TEMP_DIR}/isaac-dev-deploy.tar.gz"

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
    --exclude='dist' \
    --exclude='.setup_state' \
    --exclude='.setup.log' \
    -czf "$ARCHIVE" .

# Deploy to target
echo -e "${GREEN}Deploying to ${TARGET_HOST}...${NC}"
scp "$ARCHIVE" "${TARGET_USER}@${TARGET_HOST}:/tmp/isaac-dev-deploy.tar.gz"

# Extract on target
echo -e "${GREEN}Extracting on target...${NC}"
ssh "${TARGET_USER}@${TARGET_HOST}" << EOF
    set -e
    mkdir -p ${TARGET_DEV_DIR}
    cd ${TARGET_DEV_DIR}
    tar -xzf /tmp/isaac-dev-deploy.tar.gz
    rm /tmp/isaac-dev-deploy.tar.gz
    chmod +x setup.sh scripts/*/*.sh 2>/dev/null || true
EOF

# Optionally restart services
echo ""
echo -e "${YELLOW}Restart services? (y/N)${NC}"
read -r response
if [[ "$response" =~ ^[Yy]$ ]]; then
    echo -e "${GREEN}Restarting services...${NC}"
    ssh "${TARGET_USER}@${TARGET_HOST}" << EOF
        sudo systemctl restart isaac-robot.service || true
        sudo systemctl restart isaac-system-monitor.service || true
EOF
fi

# Cleanup
rm -rf "$TEMP_DIR"

echo ""
echo -e "${GREEN}=========================================="
echo "Deployment Complete!"
echo "==========================================${NC}"
echo ""
echo "Dev sandbox updated at: ${TARGET_DEV_DIR}"
echo "Changes are active immediately (dev takes precedence)"
echo ""
