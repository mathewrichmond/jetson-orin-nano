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

    log_step "1" "10" "Updating system packages"

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

    log_step "2" "10" "Installing Docker and NVIDIA Container Runtime"

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

    log_step "3" "10" "Setting up device permissions"

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

    log_step "4" "10" "Setting up network configuration"

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

    log_step "5" "10" "Setting up WiFi (optional)"

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

    log_step "6" "10" "Setting up Bluetooth (optional)"

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

    log_step "7" "10" "Hardware verification"

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

# Step 8: Production data structure
step_setup_production_data() {
    if check_step "setup_production_data"; then
        log "Skipping: Production data structure already created"
        return 0
    fi

    log_step "8" "10" "Setting up production data structure"

    # Detect if running from NVMe
    ROOT_DEV=$(findmnt -n -o SOURCE / 2>/dev/null)
    if [[ "$ROOT_DEV" =~ nvme ]]; then
        log "Detected NVMe boot - setting up production structure"
        
        # Create data directories
        log "Creating /data structure..."
        sudo mkdir -p /data/{logs,config,models,sessions,docker}
        sudo chown -R $USER:$USER /data
        
        # Copy configs to persistent storage
        if [ -d "$SCRIPT_DIR/config" ]; then
            log "Copying configs to /data/config..."
            mkdir -p /data/config/robot
            cp -rn "$SCRIPT_DIR/config/"* /data/config/ 2>/dev/null || true
        fi
        
        log "✓ Production data structure created at /data/"
        mark_step_complete "setup_production_data"
    else
        log "Running from SD card - skipping production data setup (run after NVMe migration)"
        mark_step_complete "setup_production_data"
    fi
}

# Step 9: Headless boot configuration
step_setup_headless_boot() {
    if check_step "setup_headless_boot"; then
        log "Skipping: Headless boot already configured"
        return 0
    fi

    log_step "9" "10" "Configuring headless boot"

    echo ""
    echo "Configure headless boot (no GUI, SSH/mDNS only)? Recommended for autonomous operation (y/N)"
    read -r response

    if [[ "$response" =~ ^[Yy]$ ]]; then
        log "Setting default target to multi-user (no GUI)..."
        sudo systemctl set-default multi-user.target
        log "✓ Headless boot configured - system will boot to multi-user mode"
        mark_step_complete "setup_headless_boot"
    else
        log "Headless boot skipped - system will use default graphical target"
        mark_step_complete "setup_headless_boot"
    fi
}

# Step 10: SD fallback and data sync
step_setup_sd_fallback_sync() {
    if check_step "setup_sd_fallback_sync"; then
        log "Skipping: SD fallback and sync already configured"
        return 0
    fi

    log_step "10" "10" "Setting up SD fallback and data sync"

    # Check if running from NVMe
    ROOT_DEV=$(findmnt -n -o SOURCE / 2>/dev/null)
    if [[ "$ROOT_DEV" =~ nvme ]]; then
        echo ""
        echo "Add SD card fallback to bootloader? Recommended for headless recovery (y/N)"
        read -r response

        if [[ "$response" =~ ^[Yy]$ ]]; then
            if [ -f "$SCRIPT_DIR/scripts/system/add_sd_fallback.sh" ]; then
                log "Adding SD fallback to bootloader..."
                sudo "$SCRIPT_DIR/scripts/system/add_sd_fallback.sh"
            else
                log "WARNING: SD fallback script not found"
            fi
        else
            log "SD fallback skipped - can add later with: sudo ./scripts/system/add_sd_fallback.sh"
        fi

        # Install sync service
        echo ""
        echo "Install data sync service (syncs logs/config hourly when network available)? (y/N)"
        read -r response

        if [[ "$response" =~ ^[Yy]$ ]]; then
            log "Installing sync service..."
            
            # Copy sync script
            sudo cp "$SCRIPT_DIR/scripts/system/isaac-sync.sh" /usr/local/bin/isaac-sync.sh
            sudo chmod +x /usr/local/bin/isaac-sync.sh
            
            # Install systemd units
            sudo cp "$SCRIPT_DIR/config/systemd/isaac-sync.service" /etc/systemd/system/
            sudo cp "$SCRIPT_DIR/config/systemd/isaac-sync.timer" /etc/systemd/system/
            
            # Enable and start timer
            sudo systemctl daemon-reload
            sudo systemctl enable isaac-sync.timer
            sudo systemctl start isaac-sync.timer
            
            log "✓ Sync service installed - configure remote server in /usr/local/bin/isaac-sync.sh"
        else
            log "Sync service skipped"
        fi
        
        mark_step_complete "setup_sd_fallback_sync"
    else
        log "Running from SD card - skipping SD fallback/sync setup (run after NVMe migration)"
        mark_step_complete "setup_sd_fallback_sync"
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
    step_setup_production_data
    step_setup_headless_boot
    step_setup_sd_fallback_sync

    log_section "Host Setup Complete!"
    
    # Detect boot device for final instructions
    ROOT_DEV=$(findmnt -n -o SOURCE / 2>/dev/null)
    
    echo ""
    echo -e "${GREEN}=== Configuration Summary ===${NC}"
    echo ""
    if [[ "$ROOT_DEV" =~ nvme ]]; then
        echo "✓ Boot device: NVMe SSD (production system)"
        [ -d "/data" ] && echo "✓ Data structure: /data/{logs,config,models,sessions}"
        echo "✓ Docker data: /data/docker"
        systemctl get-default 2>/dev/null | grep -q multi-user && echo "✓ Headless boot: Enabled (multi-user)" || echo "  Headless boot: Disabled (graphical)"
        systemctl is-enabled isaac-sync.timer &>/dev/null && echo "✓ Data sync: Enabled (hourly)" || echo "  Data sync: Not configured"
    else
        echo "  Boot device: SD card"
        echo "  Production features available after NVMe migration"
    fi
    echo ""
    
    echo -e "${GREEN}=== Next Steps ===${NC}"
    echo ""
    echo "1. ${BLUE}IMPORTANT${NC}: Reboot or logout/login for group membership:"
    echo "   sudo reboot"
    echo ""
    
    if [[ "$ROOT_DEV" =~ nvme ]]; then
        echo "2. Configure data sync (if enabled):"
        echo "   sudo nano /usr/local/bin/isaac-sync.sh"
        echo "   # Set REMOTE_USER, REMOTE_HOST, REMOTE_PATH"
        echo ""
        echo "3. Verify system:"
        echo "   make system-check"
        echo ""
        echo "4. Configure robot:"
        echo "   cp .env.example .env"
        echo "   nano .env  # Set ROBOT_ID, ZENOH_ROUTER, camera serials"
        echo ""
        echo "5. Build and start containers:"
        echo "   docker compose -f docker-compose.production.yml build"
        echo "   docker compose -f docker-compose.production.yml up -d"
        echo ""
        echo "6. Monitor:"
        echo "   docker compose logs -f"
        echo ""
        echo -e "${YELLOW}See docs/setup/PRODUCTION_SYSTEM_DESIGN.md for architecture${NC}"
    else
        echo "2. (Optional) Migrate to NVMe SSD:"
        echo "   sudo ./scripts/system/verify_ssd.sh"
        echo "   sudo ./scripts/system/migrate_to_ssd.sh"
        echo "   # Then re-run ./setup.sh for production features"
        echo ""
        echo "3. Configure robot:"
        echo "   cp .env.example .env"
        echo "   nano .env  # Set ROBOT_ID, ZENOH_ROUTER, camera serials"
        echo ""
        echo "4. Build Docker containers:"
        echo "   docker compose build"
        echo ""
        echo "5. Start robot services:"
        echo "   docker compose up -d"
        echo ""
        echo -e "${YELLOW}See docs/setup/SSD_MIGRATION.md for NVMe setup${NC}"
    fi
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
