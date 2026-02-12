#!/bin/bash
# Jetson Orin Nano - Docker-First Setup
# Minimal host setup for containerized robot system
# Installs ONLY what containers cannot provide: Docker runtime, device access, network config
#
# Architecture: Perception and control run in containers, host provides hardware access layer
# See: docs/setup/DOCKER_FIRST_SETUP.md

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
    else
        echo "ubuntu"
    fi
}

ENV_TYPE=$(detect_environment)

# Docker-first mode: skip if running in container
if [ "$ENV_TYPE" = "docker" ]; then
    echo "Running in container - host setup not needed"
    echo "See perception/ or control/ Dockerfile for container setup"
    exit 0
fi

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

    log_step "1" "7" "Updating system packages"

    if ! check_root; then
        sudo apt-get update
        sudo apt-get upgrade -y
    else
        apt-get update
        apt-get upgrade -y
    fi

    mark_step_complete "update_system"
}

# Step 2: Install Docker and NVIDIA Container Runtime
step_install_docker() {
    if check_step "install_docker"; then
        log "Skipping: Docker already installed"
        return 0
    fi

    log_step "2" "7" "Installing Docker and NVIDIA Container Runtime"

    # Install Docker
    if ! command -v docker &> /dev/null; then
        log "Installing Docker..."
        curl -fsSL https://get.docker.com -o /tmp/get-docker.sh
        sudo sh /tmp/get-docker.sh
        sudo usermod -aG docker $USER
        rm /tmp/get-docker.sh
    fi

    # Install NVIDIA Container Toolkit
    if ! dpkg -l | grep -q nvidia-docker2; then
        log "Installing NVIDIA Container Toolkit..."
        distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
        curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
        curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
            sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
            sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
        sudo apt-get update
        sudo apt-get install -y nvidia-docker2
    fi

    # Configure Docker daemon
    log "Configuring Docker daemon..."
    sudo mkdir -p /etc/docker
    sudo tee /etc/docker/daemon.json > /dev/null <<EOF
{
  "data-root": "/data/docker",
  "runtimes": {
    "nvidia": {
      "path": "nvidia-container-runtime",
      "runtimeArgs": []
    }
  },
  "default-runtime": "nvidia",
  "log-driver": "json-file",
  "log-opts": {
    "max-size": "10m",
    "max-file": "3"
  }
}
EOF

    sudo systemctl restart docker

    # Install docker-compose plugin
    if ! docker compose version &> /dev/null; then
        log "Installing Docker Compose plugin..."
        sudo apt-get install -y docker-compose-plugin
    fi

    # Test GPU access
    log "Testing GPU access in Docker..."
    if docker run --rm --gpus all nvcr.io/nvidia/l4t-base:r35.2.1 nvidia-smi &> /dev/null; then
        log "GPU access verified!"
    else
        log "WARNING: GPU test failed - may need reboot"
    fi

    mark_step_complete "install_docker"
}

# Step 3: Setup device permissions (udev rules)
step_setup_device_permissions() {
    if check_step "setup_device_permissions"; then
        log "Skipping: Device permissions already configured"
        return 0
    fi

    log_step "3" "7" "Setting up device permissions"

    # RealSense cameras
    log "Configuring RealSense camera permissions..."
    sudo tee /etc/udev/rules.d/99-realsense.rules > /dev/null <<'EOF'
# Intel RealSense cameras
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", MODE="0666", GROUP="plugdev"
KERNEL=="video[0-9]*", ATTRS{idVendor}=="8086", MODE="0666", GROUP="video"
EOF

    # Add user to required groups
    sudo usermod -aG docker,video,dialout,i2c,gpio $USER || true

    # Reload udev rules
    sudo udevadm control --reload-rules
    sudo udevadm trigger

    log "Device permissions configured. Group membership requires logout/login or reboot."

    mark_step_complete "setup_device_permissions"
}

# Step 4: Setup network configuration
step_setup_network() {
    if check_step "setup_network"; then
        log "Skipping: Network already configured"
        return 0
    fi

    log_step "4" "7" "Setting up network configuration"

    # Set hostname for mDNS
    ROBOT_ID=$(grep -E "^ROBOT_ID=" .env 2>/dev/null | cut -d= -f2 || echo "jetson-01")
    log "Setting hostname to $ROBOT_ID..."
    sudo hostnamectl set-hostname $ROBOT_ID

    # Install avahi for mDNS (allows jetson-01.local addressing)
    if ! command -v avahi-daemon &> /dev/null; then
        log "Installing Avahi for mDNS..."
        sudo apt-get install -y avahi-daemon avahi-utils
        sudo systemctl enable avahi-daemon
        sudo systemctl start avahi-daemon
    fi

    log "Network configuration complete. Robot accessible at ${ROBOT_ID}.local"

    mark_step_complete "setup_network"
}

# Step 5: Setup WiFi (optional, host-level)
step_setup_wifi() {
    if check_step "setup_wifi"; then
        log "Skipping: WiFi setup already completed"
        return 0
    fi

    log_step "5" "7" "Setting up WiFi (optional)"

    echo ""
    echo "Setup WiFi as fallback when Ethernet is disconnected? (y/N)"
    read -r response

    if [[ "$response" =~ ^[Yy]$ ]]; then
        if [ -f "$SCRIPT_DIR/scripts/system/setup_wifi.sh" ]; then
            sudo "$SCRIPT_DIR/scripts/system/setup_wifi.sh"
            mark_step_complete "setup_wifi"
        else
            log "WARNING: WiFi setup script not found"
        fi
    else
        log "Skipping WiFi setup. Run manually: sudo ./scripts/system/setup_wifi.sh"
    fi
}

# Step 6: Setup Bluetooth (optional, host-level)
step_setup_bluetooth() {
    if check_step "setup_bluetooth"; then
        log "Skipping: Bluetooth setup already completed"
        return 0
    fi

    log_step "6" "7" "Setting up Bluetooth (optional)"

    echo ""
    echo "Setup Bluetooth support? (y/N)"
    read -r response

    if [[ "$response" =~ ^[Yy]$ ]]; then
        if [ -f "$SCRIPT_DIR/scripts/system/setup_bluetooth.sh" ]; then
            "$SCRIPT_DIR/scripts/system/setup_bluetooth.sh"
            mark_step_complete "setup_bluetooth"
        else
            log "WARNING: Bluetooth setup script not found"
        fi
    else
        log "Skipping Bluetooth setup. Run manually: ./scripts/system/setup_bluetooth.sh"
    fi
}

# Step 7: Hardware verification
step_verify_hardware() {
    if check_step "verify_hardware"; then
        log "Skipping: Hardware verification already completed"
        return 0
    fi

    log_step "7" "7" "Hardware verification"

    echo ""
    echo "Run hardware verification? (checks all connected hardware) (y/N)"
    read -r response

    if [[ "$response" =~ ^[Yy]$ ]]; then
        if [ -f "$SCRIPT_DIR/scripts/hardware/setup_hardware.sh" ]; then
            "$SCRIPT_DIR/scripts/hardware/setup_hardware.sh" verify || {
                log "Hardware verification completed with warnings"
            }
        else
            log "WARNING: Hardware verification script not found"
        fi
        mark_step_complete "verify_hardware"
    else
        log "Hardware verification skipped. Run manually: ./scripts/hardware/setup_hardware.sh verify"
        mark_step_complete "verify_hardware"
    fi
}

# Check if reboot is needed
check_reboot() {
    if [ -f /var/run/reboot-required ]; then
        echo ""
        echo -e "${YELLOW}========================================${NC}"
        echo -e "${YELLOW}Reboot Required${NC}"
        echo -e "${YELLOW}========================================${NC}"
        echo ""
        echo "The system needs to reboot to complete setup."
        echo ""

        # Check if running non-interactively (from script)
        if [ "${NON_INTERACTIVE:-false}" = "true" ]; then
            echo "Rebooting in 10 seconds..."
            echo "Press Ctrl+C to cancel"
            sleep 10
            reboot
        else
            echo "Reboot now? (y/N)"
            read -r response
            if [[ "$response" =~ ^[Yy]$ ]]; then
                echo "Rebooting..."
                reboot
            else
                echo "Please reboot manually when ready: sudo reboot"
            fi
        fi
    fi
}

# Main execution
main() {
    log_section "Jetson Orin Nano - Docker-First Setup"
    log "Environment: $ENV_TYPE"
    log "Script directory: $SCRIPT_DIR"
    echo ""
    echo -e "${YELLOW}This setup installs ONLY host-level requirements.${NC}"
    echo -e "${YELLOW}Perception, control, and ROS2 run in Docker containers.${NC}"
    echo ""

    # Create setup state file if it doesn't exist
    touch "$SETUP_STATE"

    # Run setup steps
    step_update_system
    step_install_docker
    step_setup_device_permissions
    step_setup_network
    step_setup_wifi
    step_setup_bluetooth
    step_verify_hardware

    log_section "Host Setup Complete!"
    echo ""
    echo -e "${GREEN}=== Next Steps ===${NC}"
    echo ""
    echo "1. ${BLUE}IMPORTANT${NC}: Reboot or logout/login for group membership:"
    echo "   sudo reboot"
    echo ""
    echo "2. After reboot, configure robot:"
    echo "   cp .env.example .env"
    echo "   nano .env  # Set ROBOT_ID, ZENOH_ROUTER, camera serials"
    echo ""
    echo "3. Build Docker containers:"
    echo "   docker compose build"
    echo ""
    echo "4. Start robot services:"
    echo "   docker compose up -d"
    echo ""
    echo "5. Monitor logs:"
    echo "   docker compose logs -f perception control"
    echo ""
    echo "6. (Optional) Install systemd service for auto-start:"
    echo "   ./scripts/system/install_services.sh"
    echo ""
    echo -e "${YELLOW}See docs/setup/DOCKER_FIRST_SETUP.md for full guide${NC}"
    echo ""

    # Check for reboot requirement
    check_reboot

    echo ""
    echo "To reset setup state, delete: $SETUP_STATE"
}

# Run main if executed directly
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi
