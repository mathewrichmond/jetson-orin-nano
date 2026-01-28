#!/bin/bash
# Remote Deployment Script
# Deploys code to remote robot hosts

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

# Configuration
DEPLOYMENT_CONFIG="${DEPLOYMENT_CONFIG:-dual_compute}"
DRY_RUN="${DRY_RUN:-false}"
BUILD_AFTER_DEPLOY="${BUILD_AFTER_DEPLOY:-true}"
RESTART_SERVICES="${RESTART_SERVICES:-false}"

# Parse arguments
TARGET_HOST="${1:-}"
if [ -z "$TARGET_HOST" ]; then
    echo -e "${RED}Error: Target host required${NC}"
    echo "Usage: $0 <jetson|pi|all> [deployment_config]"
    exit 1
fi

DEPLOYMENT_CONFIG="${2:-$DEPLOYMENT_CONFIG}"

echo -e "${BLUE}=== Remote Deployment ===${NC}"
echo "Target: $TARGET_HOST"
echo "Deployment: $DEPLOYMENT_CONFIG"
echo "Dry run: $DRY_RUN"
echo ""

# Load deployment configuration
DEPLOY_CONFIG_FILE="$REPO_ROOT/config/deployment/${DEPLOYMENT_CONFIG}.yaml"
if [ ! -f "$DEPLOY_CONFIG_FILE" ]; then
    echo -e "${RED}Error: Deployment config not found: $DEPLOY_CONFIG_FILE${NC}"
    exit 1
fi

# Deploy to single host
deploy_to_host() {
    local host="$1"
    local remote_user="$2"
    local remote_host="$3"
    local packages="$4"
    
    echo -e "${GREEN}Deploying to $host ($remote_user@$remote_host)${NC}"
    
    # Test SSH connection
    echo "Testing SSH connection..."
    if ! ssh -o ConnectTimeout=5 "$remote_user@$remote_host" "echo 'Connection OK'"; then
        echo -e "${RED}Error: Cannot connect to $remote_host${NC}"
        return 1
    fi
    
    # Rsync code
    echo "Syncing code..."
    
    RSYNC_OPTS=(
        -avz
        --delete
        --exclude '.git'
        --exclude 'build'
        --exclude 'install'
        --exclude 'log'
        --exclude '.venv'
        --exclude '__pycache__'
        --exclude '*.pyc'
        --exclude '.pytest_cache'
        --exclude 'htmlcov'
    )
    
    if [ "$DRY_RUN" = "true" ]; then
        RSYNC_OPTS+=(--dry-run)
    fi
    
    rsync "${RSYNC_OPTS[@]}" \
        "$REPO_ROOT/" \
        "$remote_user@$remote_host:~/src/jetson-orin-nano/"
    
    if [ "$?" -ne 0 ]; then
        echo -e "${RED}Error: rsync failed${NC}"
        return 1
    fi
    
    echo -e "${GREEN}Code synced successfully${NC}"
    
    # Build on remote
    if [ "$BUILD_AFTER_DEPLOY" = "true" ] && [ "$DRY_RUN" = "false" ]; then
        echo "Building on remote host..."
        
        ssh "$remote_user@$remote_host" << EOF
            cd ~/src/jetson-orin-nano
            
            # Source ROS 2
            source /opt/ros/humble/setup.bash || true
            
            # Build packages
            if [ -n "$packages" ]; then
                echo "Building packages: $packages"
                colcon build --packages-select $packages --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
            else
                echo "Building all packages"
                colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
            fi
            
            echo "Build completed"
EOF
        
        if [ "$?" -ne 0 ]; then
            echo -e "${RED}Error: Build failed on remote host${NC}"
            return 1
        fi
        
        echo -e "${GREEN}Build completed successfully${NC}"
    fi
    
    # Restart services
    if [ "$RESTART_SERVICES" = "true" ] && [ "$DRY_RUN" = "false" ]; then
        echo "Restarting services..."
        
        ssh "$remote_user@$remote_host" << EOF
            # Stop service
            systemctl --user stop isaac-robot.service || true
            
            # Wait a moment
            sleep 2
            
            # Start service
            systemctl --user start isaac-robot.service || true
            
            # Check status
            systemctl --user status isaac-robot.service || true
EOF
    fi
    
    echo -e "${GREEN}Deployment to $host complete${NC}"
    echo ""
}

# Deploy based on target
case "$TARGET_HOST" in
    jetson)
        # Get Jetson configuration
        JETSON_USER="${JETSON_USER:-nano}"
        JETSON_HOST="${JETSON_HOST:-isaac-jetson.local}"
        JETSON_PACKAGES="custom_msgs vision_pipeline audio_pipeline vla_planner power_management"
        
        deploy_to_host "jetson" "$JETSON_USER" "$JETSON_HOST" "$JETSON_PACKAGES"
        ;;
    
    pi)
        # Get Pi configuration
        PI_USER="${PI_USER:-pi}"
        PI_HOST="${PI_HOST:-isaac-pi.local}"
        PI_PACKAGES="custom_msgs chassis_control power_management"
        
        deploy_to_host "pi" "$PI_USER" "$PI_HOST" "$PI_PACKAGES"
        ;;
    
    all)
        echo -e "${YELLOW}Deploying to all hosts${NC}"
        echo ""
        
        # Deploy to Jetson
        JETSON_USER="${JETSON_USER:-nano}"
        JETSON_HOST="${JETSON_HOST:-isaac-jetson.local}"
        JETSON_PACKAGES="custom_msgs vision_pipeline audio_pipeline vla_planner power_management"
        deploy_to_host "jetson" "$JETSON_USER" "$JETSON_HOST" "$JETSON_PACKAGES"
        
        # Deploy to Pi
        PI_USER="${PI_USER:-pi}"
        PI_HOST="${PI_HOST:-isaac-pi.local}"
        PI_PACKAGES="custom_msgs chassis_control power_management"
        deploy_to_host "pi" "$PI_USER" "$PI_HOST" "$PI_PACKAGES"
        ;;
    
    *)
        echo -e "${RED}Unknown target: $TARGET_HOST${NC}"
        echo "Valid targets: jetson, pi, all"
        exit 1
        ;;
esac

echo -e "${BLUE}=== Deployment Complete ===${NC}"
echo ""
echo "Next steps:"
echo "1. Verify deployment: ssh <user>@<host> 'ros2 node list'"
echo "2. Check logs: ssh <user>@<host> 'journalctl --user -u isaac-robot.service -f'"
echo "3. Test system: ssh <user>@<host> 'cd ~/src/jetson-orin-nano && ./scripts/testing/run_tests.sh'"
