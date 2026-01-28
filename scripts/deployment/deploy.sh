#!/bin/bash
# Deployment Script
# Deploys Isaac robot system to different environments

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Parse arguments
DEPLOYMENT="${1:-local}"
ACTION="${2:-launch}"

echo -e "${GREEN}=== Isaac Robot Deployment ===${NC}"
echo "Deployment: $DEPLOYMENT"
echo "Action: $ACTION"
echo ""

# Load deployment configuration
DEPLOYMENT_CONFIG="$REPO_ROOT/config/deployment/${DEPLOYMENT}.yaml"

if [ ! -f "$DEPLOYMENT_CONFIG" ]; then
    echo "Error: Deployment configuration not found: $DEPLOYMENT_CONFIG"
    exit 1
fi

case "$ACTION" in
    setup)
        echo "Setting up deployment environment..."
        
        # Run network setup if required
        if grep -q "require_sudo: true" "$DEPLOYMENT_CONFIG"; then
            echo "This deployment requires network configuration with sudo"
            sudo "$REPO_ROOT/scripts/network/setup_network.sh" "$DEPLOYMENT"
        fi
        
        # Source ROS environment
        if [ -f "$REPO_ROOT/install/setup.bash" ]; then
            source "$REPO_ROOT/install/setup.bash"
        fi
        
        echo -e "${GREEN}Setup complete${NC}"
        ;;
    
    launch)
        echo "Launching deployment..."
        
        # Source ROS environment
        if [ -f "$REPO_ROOT/install/setup.bash" ]; then
            source "$REPO_ROOT/install/setup.bash"
        fi
        
        case "$DEPLOYMENT" in
            local)
                ros2 launch isaac_robot graph.launch.py \
                    graph:=modular_graph.yaml \
                    group:=all \
                    deployment:=local
                ;;
            
            dual_compute)
                # Auto-detect host
                HOSTNAME=$(hostname)
                ros2 launch isaac_robot distributed.launch.py \
                    deployment:=dual_compute \
                    host:=auto
                ;;
            
            test)
                ros2 launch isaac_robot test.launch.py \
                    mock_hardware:=true \
                    headless:=true
                ;;
            
            sim)
                echo "Launching simulation environment..."
                # Would launch Gazebo/Isaac Sim first
                ros2 launch isaac_robot graph.launch.py \
                    graph:=modular_graph.yaml \
                    group:=all \
                    deployment:=sim \
                    use_sim_time:=true
                ;;
            
            cloud_dev)
                echo "Launching cloud development environment..."
                ros2 launch isaac_robot distributed.launch.py \
                    deployment:=cloud_dev \
                    host:=auto
                ;;
            
            *)
                echo "Unknown deployment: $DEPLOYMENT"
                exit 1
                ;;
        esac
        ;;
    
    stop)
        echo "Stopping deployment..."
        ros2 daemon stop
        pkill -f "ros2 launch"
        echo -e "${GREEN}Deployment stopped${NC}"
        ;;
    
    status)
        echo "Checking deployment status..."
        ros2 node list
        ros2 topic list
        ;;
    
    *)
        echo "Unknown action: $ACTION"
        echo "Usage: $0 <deployment> <action>"
        echo "  deployment: local, dual_compute, test, sim, cloud_dev"
        echo "  action: setup, launch, stop, status"
        exit 1
        ;;
esac
