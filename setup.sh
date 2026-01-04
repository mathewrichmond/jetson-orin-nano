#!/bin/bash
# Unified Setup Script
# Single entry point for setting up the Isaac robot system
# Works for both native Jetson and Docker environments

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m'

# Configuration
SETUP_LOG="${SETUP_LOG:-$SCRIPT_DIR/.setup.log}"
SETUP_STATE="${SETUP_STATE:-$SCRIPT_DIR/.setup_state}"

# Detect environment
detect_environment() {
    if [ -f /.dockerenv ] || grep -qa docker /proc/1/cgroup 2>/dev/null; then
        echo "docker"
    elif [ -f /etc/nv_tegra_release ]; then
        echo "jetson"
    elif [ -f /etc/os-release ] && grep -q "Ubuntu" /etc/os-release; then
        echo "ubuntu"
    else
        echo "unknown"
    fi
}

ENV_TYPE=$(detect_environment)

log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $*" | tee -a "$SETUP_LOG"
}

log_section() {
    echo ""
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}$*${NC}"
    echo -e "${BLUE}========================================${NC}"
    log "SECTION: $*"
}

log_step() {
    echo -e "${GREEN}[$1/$2] $3${NC}"
    log "STEP [$1/$2]: $3"
}

check_step() {
    local step_name="$1"
    if [ -f "$SETUP_STATE" ] && grep -q "^$step_name$" "$SETUP_STATE"; then
        return 0
    fi
    return 1
}

mark_step_complete() {
    local step_name="$1"
    echo "$step_name" >> "$SETUP_STATE"
    log "COMPLETED: $step_name"
}

# Check if running as root (for system packages)
check_root() {
    if [ "$EUID" -ne 0 ]; then
        echo "This step requires root privileges. Using sudo..."
        return 1
    fi
    return 0
}

# Step 1: Update system packages
step_update_system() {
    if check_step "update_system"; then
        log "Skipping: System update already completed"
        return 0
    fi

    log_step "1" "6" "Updating system packages"

    if ! check_root; then
        sudo apt-get update
        sudo apt-get upgrade -y
    else
        apt-get update
        apt-get upgrade -y
    fi

    mark_step_complete "update_system"
}

# Step 2: Install system packages
step_install_system_packages() {
    if check_step "install_system_packages"; then
        log "Skipping: System packages already installed"
        return 0
    fi

    log_step "2" "6" "Installing system packages"

    # Check if package manager exists
    if [ ! -f "$SCRIPT_DIR/scripts/utils/package_manager.py" ]; then
        log "ERROR: Package manager not found"
        exit 1
    fi

    # Install PyYAML if needed
    if ! python3 -c "import yaml" 2>/dev/null; then
        log "Installing PyYAML..."
        pip3 install --user pyyaml || sudo pip3 install pyyaml
    fi

    # Install packages based on environment
    if [ "$ENV_TYPE" = "docker" ]; then
        # Docker: Install minimal set
        python3 "$SCRIPT_DIR/scripts/utils/package_manager.py" install-system --groups dev_minimal
    elif [ "$ENV_TYPE" = "jetson" ]; then
        # Jetson: Install robot essentials
        python3 "$SCRIPT_DIR/scripts/utils/package_manager.py" install-system --groups robot_essentials
    else
        # Other Ubuntu: Install full dev environment
        python3 "$SCRIPT_DIR/scripts/utils/package_manager.py" install-system --groups dev_full
    fi

    mark_step_complete "install_system_packages"
}

# Step 3: Install Python packages
step_install_python_packages() {
    if check_step "install_python_packages"; then
        log "Skipping: Python packages already installed"
        return 0
    fi

    log_step "3" "6" "Installing Python packages"

    # Upgrade pip
    pip3 install --upgrade pip setuptools wheel --user || sudo pip3 install --upgrade pip setuptools wheel

    # Install PyYAML if needed
    if ! python3 -c "import yaml" 2>/dev/null; then
        pip3 install --user pyyaml || sudo pip3 install pyyaml
    fi

    # Install Python packages
    if [ "$ENV_TYPE" = "docker" ]; then
        python3 "$SCRIPT_DIR/scripts/utils/package_manager.py" install-python --groups dev_minimal
    else
        python3 "$SCRIPT_DIR/scripts/utils/package_manager.py" install-python --groups dev_all
    fi

    # Install project requirements if they exist
    if [ -f "$SCRIPT_DIR/requirements-dev.txt" ]; then
        log "Installing project requirements..."
        pip3 install --user -r "$SCRIPT_DIR/requirements-dev.txt" || true
    fi

    mark_step_complete "install_python_packages"
}

# Step 4: Setup ROS 2 workspace
step_setup_ros2_workspace() {
    if check_step "setup_ros2_workspace"; then
        log "Skipping: ROS 2 workspace already set up"
        return 0
    fi

    log_step "4" "6" "Setting up ROS 2 workspace"

    # Check if ROS 2 is installed
    if [ ! -f "/opt/ros/humble/setup.bash" ]; then
        log "WARNING: ROS 2 Humble not found. Installing..."
        if ! check_root; then
            sudo "$SCRIPT_DIR/scripts/system/setup_isaac.sh" || true
        fi
    fi

    # Source ROS 2
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash

        # Initialize workspace
        mkdir -p ~/ros2_ws/src
        cd ~/ros2_ws

        if [ ! -f .rosinstall ]; then
            if command -v wstool &> /dev/null; then
                wstool init src || true
            fi
        fi

        # Link system_monitor if it exists
        if [ -d "$SCRIPT_DIR/src/system_monitor" ]; then
            cd ~/ros2_ws/src
            ln -sf "$SCRIPT_DIR/src/system_monitor" system_monitor 2>/dev/null || true
        fi

        # Install dependencies
        if command -v rosdep &> /dev/null; then
            cd ~/ros2_ws
            rosdep update || true
            rosdep install --from-paths src --ignore-src -r -y || true
        fi

        # Build workspace
        log "Building ROS 2 workspace..."
        colcon build --symlink-install || log "Build completed with warnings"
    else
        log "WARNING: ROS 2 not available, skipping workspace setup"
    fi

    mark_step_complete "setup_ros2_workspace"
}

# Step 5: Setup Python virtual environment
step_setup_venv() {
    if check_step "setup_venv"; then
        log "Skipping: Virtual environment already set up"
        return 0
    fi

    log_step "5" "7" "Setting up Python virtual environment"

    if [ ! -d "$SCRIPT_DIR/.venv" ]; then
        python3 -m venv "$SCRIPT_DIR/.venv"
        source "$SCRIPT_DIR/.venv/bin/activate"
        pip install --upgrade pip setuptools wheel

        # Install requirements
        if [ -f "$SCRIPT_DIR/requirements-dev.txt" ]; then
            pip install -r "$SCRIPT_DIR/requirements-dev.txt"
        fi

        deactivate
    fi

    mark_step_complete "setup_venv"
}

# Step 6: Install pre-commit hooks
step_install_precommit() {
    if check_step "install_precommit"; then
        log "Skipping: Pre-commit hooks already installed"
        return 0
    fi

    log_step "6" "7" "Installing pre-commit hooks"

    if [ -d "$SCRIPT_DIR/.git" ] && [ -f "$SCRIPT_DIR/.pre-commit-config.yaml" ]; then
        if command -v pre-commit &> /dev/null; then
            cd "$SCRIPT_DIR"
            pre-commit install || log "Pre-commit installation failed (may need to install pre-commit)"
        else
            log "Pre-commit not found, skipping hook installation"
        fi
    else
        log "Not a git repository or pre-commit config not found, skipping"
    fi

    mark_step_complete "install_precommit"
}

# Step 7: Install systemd services (optional, only on Jetson)
step_install_services() {
    if [ "$ENV_TYPE" != "jetson" ]; then
        return 0
    fi

    if check_step "install_services"; then
        log "Skipping: Services already installed"
        return 0
    fi

    log_step "7" "7" "Installing systemd services (optional)"
    echo ""
    echo "Install systemd services for auto-start and maintenance? (y/N)"
    read -r response
    if [[ "$response" =~ ^[Yy]$ ]]; then
        if [ "$EUID" -eq 0 ]; then
            "$SCRIPT_DIR/scripts/system/install_services.sh"
        else
            sudo "$SCRIPT_DIR/scripts/system/install_services.sh"
        fi
        mark_step_complete "install_services"
    else
        log "Skipping service installation. Run manually: sudo ./scripts/system/install_services.sh"
    fi
}

# Main execution
main() {
    log_section "Isaac Robot System Setup"
    log "Environment: $ENV_TYPE"
    log "Script directory: $SCRIPT_DIR"

    # Create setup state file if it doesn't exist
    touch "$SETUP_STATE"

    # Run setup steps
    step_update_system
    step_install_system_packages
    step_install_python_packages
    step_setup_ros2_workspace
    step_setup_venv
    step_install_precommit
    step_install_services

    log_section "Setup Complete!"
    echo ""
    echo "Next steps:"
    echo "1. Activate virtual environment:"
    echo "   source .venv/bin/activate"
    echo ""
    echo "2. Source ROS 2 (if available):"
    echo "   source /opt/ros/humble/setup.bash"
    echo "   source ~/ros2_ws/install/setup.bash"
    echo ""
    echo "3. Run system monitor:"
    echo "   ros2 launch system_monitor system_monitor.launch.py"
    echo ""
    if [ "$ENV_TYPE" = "jetson" ]; then
        echo "4. Install auto-start services (optional):"
        echo "   sudo ./scripts/system/install_services.sh"
        echo ""
        echo "   This enables:"
        echo "   - Auto-start on boot"
        echo "   - Daily updates (3 AM)"
        echo "   - Weekly cleanup (Sunday 2 AM)"
        echo "   - Health checks (every 15 minutes)"
        echo ""
    fi
    echo "To reset setup state, delete: $SETUP_STATE"
}

# Run main if executed directly
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi
