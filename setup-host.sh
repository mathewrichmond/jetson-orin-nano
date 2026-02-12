#!/bin/bash
# Jetson Orin Nano - Minimal Host Setup for Containerized Robot System
# Installs ONLY what containers cannot provide: drivers, Docker, GPU config, network, PTP
#
# Architecture: All robot code runs in containers. Host provides hardware access layer.
# See: docs/setup/CONTAINER_FIRST_ARCHITECTURE.md

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
SETUP_STATE="${SETUP_STATE:-$SCRIPT_DIR/.setup_state_host}"

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

# Exit if running in container
if [ "$ENV_TYPE" = "docker" ]; then
    echo "Running in container - host setup not needed"
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

# Step 1: System updates and essential packages
step_system_updates() {
    if check_step "system_updates"; then
        log "Skipping: System updates already completed"
        return 0
    fi

    log_step "1" "8" "System updates and essential packages"

    sudo apt-get update
    sudo apt-get upgrade -y
    sudo apt-get install -y \
        curl \
        wget \
        git \
        build-essential \
        usbutils \
        pciutils \
        net-tools \
        iperf3 \
        htop \
        vim

    mark_step_complete "system_updates"
}

# Step 2: Docker and NVIDIA Container Runtime
step_install_docker() {
    if check_step "install_docker"; then
        log "Skipping: Docker already installed"
        return 0
    fi

    log_step "2" "8" "Installing Docker and NVIDIA Container Runtime"

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

    # Configure Docker daemon for production
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
  },
  "storage-driver": "overlay2",
  "live-restore": true
}
EOF

    sudo systemctl restart docker

    # Install docker-compose plugin
    if ! docker compose version &> /dev/null; then
        log "Installing Docker Compose plugin..."
        sudo apt-get install -y docker-compose-plugin
    fi

    mark_step_complete "install_docker"
}

# Step 3: Camera and sensor drivers (host-level, not ROS)
step_install_drivers() {
    if check_step "install_drivers"; then
        log "Skipping: Drivers already installed"
        return 0
    fi

    log_step "3" "8" "Installing camera and sensor drivers"

    # RealSense SDK (system driver, not ROS package)
    if ! dpkg -l | grep -q librealsense2; then
        log "Installing Intel RealSense SDK..."
        sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || true
        sudo add-apt-repository -y "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main"
        sudo apt-get update
        sudo apt-get install -y librealsense2-utils librealsense2-dev
    fi

    # udev rules for cameras
    log "Configuring camera permissions..."
    sudo tee /etc/udev/rules.d/99-realsense.rules > /dev/null <<'EOF'
# Intel RealSense cameras
SUBSYSTEM=="usb", ATTRS{idVendor}=="8086", MODE="0666", GROUP="plugdev"
KERNEL=="video[0-9]*", ATTRS{idVendor}=="8086", MODE="0666", GROUP="video"
EOF

    # Add user to required groups
    sudo usermod -aG docker,video,dialout,i2c,gpio,plugdev $USER || true

    # Reload udev rules
    sudo udevadm control --reload-rules
    sudo udevadm trigger

    mark_step_complete "install_drivers"
}

# Step 4: GPU performance mode
step_configure_gpu() {
    if check_step "configure_gpu"; then
        log "Skipping: GPU already configured"
        return 0
    fi

    log_step "4" "8" "Configuring GPU for high performance"

    # Set maximum performance mode
    log "Setting Jetson to MAXN power mode..."
    sudo nvpmodel -m 0 2>/dev/null || log "Warning: nvpmodel not available"

    # Set GPU to max clocks
    log "Setting GPU to maximum clocks..."
    sudo jetson_clocks 2>/dev/null || log "Warning: jetson_clocks not available"

    # Create systemd service to set performance on boot
    sudo tee /etc/systemd/system/jetson-performance.service > /dev/null <<'EOF'
[Unit]
Description=Jetson Performance Mode
After=multi-user.target

[Service]
Type=oneshot
ExecStart=/usr/sbin/nvpmodel -m 0
ExecStart=/usr/bin/jetson_clocks
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

    sudo systemctl daemon-reload
    sudo systemctl enable jetson-performance.service

    mark_step_complete "configure_gpu"
}

# Step 5: PTP (Precision Time Protocol) for time synchronization
step_setup_ptp() {
    if check_step "setup_ptp"; then
        log "Skipping: PTP already configured"
        return 0
    fi

    log_step "5" "8" "Setting up PTP for time synchronization"

    # Install PTP daemon
    sudo apt-get install -y linuxptp

    # Configure PTP
    sudo tee /etc/linuxptp/ptp4l.conf > /dev/null <<'EOF'
[global]
slaveOnly               1
priority1               255
priority2               255
domainNumber            0
clockAccuracy           0xFE
offsetScaledLogVariance 0xFFFF
free_running            0
freq_est_interval       1
dscp_event              46
dscp_general            34
dataset_comparison      ieee1588
G.8275.defaultDS.localPriority  128
logAnnounceInterval     1
logSyncInterval         0
logMinDelayReqInterval  0
announceReceiptTimeout  3
syncReceiptTimeout      0
delay_mechanism         E2E
time_stamping           hardware
network_transport       UDPv4
udp_ttl                 1
masterOnly              0
G.8275.portDS.localPriority     128
hybrid_e2e              1
inhibit_multicast_service   0
unicast_listen          0
unicast_master_table    0
logging_level           6
verbose                 0
use_syslog              1
summary_interval        0
kernel_leap             1
check_fup_sync          0
EOF

    # Create PTP systemd service
    sudo tee /etc/systemd/system/ptp4l.service > /dev/null <<'EOF'
[Unit]
Description=Precision Time Protocol (PTP) daemon
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
ExecStart=/usr/sbin/ptp4l -f /etc/linuxptp/ptp4l.conf -i eth0
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

    # PHC2SYS to sync system clock with PTP
    sudo tee /etc/systemd/system/phc2sys.service > /dev/null <<'EOF'
[Unit]
Description=Synchronize system clock to PTP hardware clock
After=ptp4l.service
Requires=ptp4l.service

[Service]
Type=simple
ExecStart=/usr/sbin/phc2sys -s eth0 -c CLOCK_REALTIME -w -O 0
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

    sudo systemctl daemon-reload
    sudo systemctl enable ptp4l.service phc2sys.service

    log "PTP configured - will start on next boot"
    log "To start now: sudo systemctl start ptp4l.service phc2sys.service"

    mark_step_complete "setup_ptp"
}

# Step 6: Network configuration
step_setup_network() {
    if check_step "setup_network"; then
        log "Skipping: Network already configured"
        return 0
    fi

    log_step "6" "8" "Setting up network configuration"

    # Set hostname for mDNS
    ROBOT_ID=$(grep -E "^ROBOT_ID=" .env 2>/dev/null | cut -d= -f2 | tr -d ' "' || echo "")
    if [ -z "$ROBOT_ID" ]; then
        ROBOT_ID="isaac"
    fi
    log "Setting hostname to $ROBOT_ID..."
    sudo hostnamectl set-hostname "$ROBOT_ID"

    # Install avahi for mDNS
    if ! command -v avahi-daemon &> /dev/null; then
        log "Installing Avahi for mDNS..."
        sudo apt-get install -y avahi-daemon avahi-utils
        sudo systemctl enable avahi-daemon
        sudo systemctl start avahi-daemon
    fi

    log "Network configuration complete. Robot accessible at ${ROBOT_ID}.local"

    mark_step_complete "setup_network"
}

# Step 7: Production data structure
step_setup_production_data() {
    if check_step "setup_production_data"; then
        log "Skipping: Production data structure already created"
        return 0
    fi

    log_step "7" "8" "Setting up production data structure"

    # Detect if running from NVMe
    ROOT_DEV=$(findmnt -n -o SOURCE / 2>/dev/null)
    if [[ "$ROOT_DEV" =~ nvme ]]; then
        log "Creating /data structure..."
        sudo mkdir -p /data/{logs,config,models,sessions,docker,worldgraph}
        sudo mkdir -p /data/config/{calibration,calibration_history,calibration_sessions}
        sudo chown -R $USER:$USER /data
        
        # Copy configs to persistent storage
        if [ -d "$SCRIPT_DIR/config" ]; then
            mkdir -p /data/config
            cp -rn "$SCRIPT_DIR/config/"* /data/config/ 2>/dev/null || true
        fi
        
        log "✓ Production data structure created at /data/"
    else
        log "Running from SD card - production data setup will run after NVMe migration"
    fi

    mark_step_complete "setup_production_data"
}

# Step 8: Container deployment system
step_setup_container_deploy() {
    if check_step "setup_container_deploy"; then
        log "Skipping: Container deployment already configured"
        return 0
    fi

    log_step "8" "8" "Setting up container deployment system"

    # Create container management script
    if [ ! -f "$SCRIPT_DIR/scripts/deployment/deploy_containers.sh" ]; then
        log "Container deployment script will be created by documentation update"
    fi

    # Install systemd service for container management
    sudo tee /etc/systemd/system/isaac-containers.service > /dev/null <<EOF
[Unit]
Description=Isaac Robot Container Stack
After=docker.service network-online.target
Requires=docker.service
Wants=network-online.target

[Service]
Type=oneshot
RemainAfterExit=yes
WorkingDirectory=$SCRIPT_DIR
ExecStart=/usr/bin/docker compose -f docker-compose.production.yml up -d
ExecStop=/usr/bin/docker compose -f docker-compose.production.yml down
User=$USER

[Install]
WantedBy=multi-user.target
EOF

    sudo systemctl daemon-reload
    
    log "Container deployment configured"
    log "Enable auto-start: sudo systemctl enable isaac-containers.service"

    mark_step_complete "setup_container_deploy"
}

# Main execution
main() {
    log_section "Jetson Orin Nano - Minimal Host Setup (Container-First)"
    log "Environment: $ENV_TYPE"
    echo ""
    echo -e "${YELLOW}This setup installs ONLY host-level requirements:${NC}"
    echo -e "${YELLOW}- System drivers (cameras, sensors)${NC}"
    echo -e "${YELLOW}- Docker + NVIDIA Container Runtime${NC}"
    echo -e "${YELLOW}- GPU performance configuration${NC}"
    echo -e "${YELLOW}- PTP time synchronization${NC}"
    echo -e "${YELLOW}- Network (mDNS)${NC}"
    echo -e "${YELLOW}- Production data structure${NC}"
    echo ""
    echo -e "${GREEN}All robot code runs in containers.${NC}"
    echo ""

    # Create setup state file if it doesn't exist
    touch "$SETUP_STATE"

    # Run setup steps
    step_system_updates
    step_install_docker
    step_install_drivers
    step_configure_gpu
    step_setup_ptp
    step_setup_network
    step_setup_production_data
    step_setup_container_deploy

    log_section "Host Setup Complete!"
    
    # Detect boot device for final instructions
    ROOT_DEV=$(findmnt -n -o SOURCE / 2>/dev/null)
    
    echo ""
    echo -e "${GREEN}=== Configuration Summary ===${NC}"
    echo ""
    echo "✓ Docker + NVIDIA runtime installed"
    echo "✓ Camera drivers installed"
    echo "✓ GPU set to high performance mode"
    echo "✓ PTP time synchronization configured"
    echo "✓ Network (mDNS) configured"
    if [[ "$ROOT_DEV" =~ nvme ]]; then
        echo "✓ Production data: /data/{logs,config,models,sessions,docker}"
    fi
    echo "✓ Container deployment system ready"
    echo ""
    
    echo -e "${GREEN}=== Next Steps ===${NC}"
    echo ""
    echo "1. ${BLUE}IMPORTANT${NC}: Reboot for group membership and GPU settings:"
    echo "   sudo reboot"
    echo ""
    echo "2. After reboot, build containers:"
    echo "   docker compose -f docker-compose.production.yml build"
    echo ""
    echo "3. Deploy containers:"
    echo "   docker compose -f docker-compose.production.yml up -d"
    echo ""
    echo "4. Enable auto-start on boot:"
    echo "   sudo systemctl enable isaac-containers.service"
    echo ""
    echo "5. Monitor containers:"
    echo "   docker compose logs -f"
    echo ""
    echo -e "${YELLOW}See docs/setup/CONTAINER_FIRST_ARCHITECTURE.md for details${NC}"
    echo ""

    # Check for reboot requirement
    if [ -f /var/run/reboot-required ] || ! groups | grep -q docker; then
        echo ""
        echo -e "${YELLOW}========================================${NC}"
        echo -e "${YELLOW}Reboot Required${NC}"
        echo -e "${YELLOW}========================================${NC}"
        echo ""
        echo "The system needs to reboot to:"
        echo "  - Apply group membership (docker, video, dialout)"
        echo "  - Enable GPU performance settings"
        echo "  - Start PTP time synchronization"
        echo ""
        echo "Reboot now? (y/N)"
        read -r response
        if [[ "$response" =~ ^[Yy]$ ]]; then
            echo "Rebooting..."
            sudo reboot
        else
            echo "Please reboot manually when ready: sudo reboot"
        fi
    fi

    echo ""
    echo "To reset setup state, delete: $SETUP_STATE"
}

# Run main if executed directly
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    main "$@"
fi
