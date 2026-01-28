#!/bin/bash
# Deploy branch to test workspace (non-production)

set -e

BRANCH_NAME="${1:-$(git rev-parse --abbrev-ref HEAD)}"
TARGET_USER="${2:-nano}"
TARGET_HOST="${3:-localhost}"
TEST_WORKSPACE="/home/$TARGET_USER/src/jetson-orin-nano-dev"

echo "═══════════════════════════════════════════════════════════════"
echo "  BRANCH DEPLOYMENT (TEST)"
echo "═══════════════════════════════════════════════════════════════"
echo "  Branch: $BRANCH_NAME"
echo "  Target: $TARGET_USER@$TARGET_HOST"
echo "  Workspace: $TEST_WORKSPACE"
echo ""

# Ensure branch is clean
if [ -n "$(git status --porcelain)" ]; then
    echo "⚠️  Warning: Working directory has uncommitted changes"
    read -p "Continue anyway? (y/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Deploy to remote
if [ "$TARGET_HOST" = "localhost" ]; then
    echo "Deploying to local test workspace..."
    
    # Create test workspace
    mkdir -p "$TEST_WORKSPACE"
    
    # Rsync current code
    rsync -av --exclude='.git' --exclude='build' --exclude='install' --exclude='log' \
        --exclude='*.bag' --exclude='__pycache__' \
        ./ "$TEST_WORKSPACE/"
    
    echo "  ✓ Code deployed locally"
else
    echo "Deploying to remote test workspace..."
    
    # Rsync to remote
    rsync -av --exclude='.git' --exclude='build' --exclude='install' --exclude='log' \
        --exclude='*.bag' --exclude='__pycache__' \
        ./ "$TARGET_USER@$TARGET_HOST:$TEST_WORKSPACE/"
    
    echo "  ✓ Code deployed to remote"
fi

# Build on target
echo ""
echo "Building test workspace..."
if [ "$TARGET_HOST" = "localhost" ]; then
    cd "$TEST_WORKSPACE"
    colcon build --symlink-install 2>&1 | tail -20
    BUILD_STATUS=$?
else
    ssh "$TARGET_USER@$TARGET_HOST" "cd $TEST_WORKSPACE && colcon build --symlink-install" 2>&1 | tail -20
    BUILD_STATUS=$?
fi

if [ $BUILD_STATUS -eq 0 ]; then
    echo "  ✓ Build successful"
else
    echo "  ❌ Build failed"
    exit 1
fi

# Restart test service
echo ""
echo "Restarting test service..."
if [ "$TARGET_HOST" = "localhost" ]; then
    systemctl --user restart isaac-robot-test.service 2>/dev/null || echo "  Service not running"
else
    ssh "$TARGET_USER@$TARGET_HOST" "systemctl --user restart isaac-robot-test.service" 2>/dev/null || echo "  Service not running"
fi

echo ""
echo "✓ Branch deployment complete"
echo "  Branch: $BRANCH_NAME"
echo "  Workspace: $TEST_WORKSPACE"
echo ""
echo "To start test service:"
echo "  systemctl --user start isaac-robot-test.service"
echo ""
echo "To view logs:"
echo "  journalctl --user -u isaac-robot-test.service -f"
echo "═══════════════════════════════════════════════════════════════"
