#!/bin/bash
# Comprehensive System Bringup Script for Jetson Orin Nano (Isaac)
# Run this script after first boot

set -e  # Exit on error

echo "=========================================="
echo "Isaac Jetson Orin Nano System Bringup"
echo "=========================================="
echo ""

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if running as root
if [ "$EUID" -ne 0 ]; then 
    echo "Please run as root or with sudo"
    exit 1
fi

# Step 1: Set hostname
echo -e "${GREEN}[1/7] Setting hostname to 'isaac'...${NC}"
hostnamectl set-hostname isaac
echo "127.0.0.1 isaac isaac.local" >> /etc/hosts
sed -i 's/127.0.1.1\tubuntu/127.0.1.1\tisaac isaac.local/' /etc/hosts
echo "✓ Hostname set to 'isaac'"

# Step 2: Configure Avahi (mDNS) for hostname resolution
echo -e "${GREEN}[2/7] Configuring Avahi mDNS for hostname resolution...${NC}"
# Enable workstation publishing in avahi
sed -i 's/#publish-workstation=no/publish-workstation=yes/' /etc/avahi/avahi-daemon.conf
# Set hostname in avahi config (optional, will use system hostname by default)
# Uncomment and set if needed: sed -i 's/#host-name=foo/host-name=isaac/' /etc/avahi/avahi-daemon.conf
systemctl restart avahi-daemon
echo "✓ Avahi configured and restarted"

# Step 3: Verify NetworkManager is using DHCP
echo -e "${GREEN}[3/7] Verifying NetworkManager DHCP configuration...${NC}"
# Check current network configuration
if nmcli connection show | grep -q "ethernet\|wifi"; then
    echo "NetworkManager is managing network connections"
    # Ensure DHCP is enabled (should be default)
    nmcli connection modify "$(nmcli -t -f NAME connection show --active | head -1)" ipv4.method auto 2>/dev/null || true
    echo "✓ NetworkManager configured for DHCP (dynamic IP)"
else
    echo "⚠ No active NetworkManager connections found"
fi

# Step 4: System updates
echo -e "${GREEN}[4/7] Updating system packages...${NC}"
apt-get update
apt-get upgrade -y
apt-get dist-upgrade -y
apt-get autoremove -y
apt-get autoclean
echo "✓ System updated"

# Step 5: Install development tools
echo -e "${GREEN}[5/7] Installing development tools...${NC}"
apt-get install -y \
    build-essential \
    cmake \
    git \
    curl \
    wget \
    vim \
    nano \
    htop \
    tree \
    net-tools \
    iputils-ping \
    openssh-server \
    python3-pip \
    python3-dev \
    python3-venv \
    software-properties-common \
    apt-transport-https \
    ca-certificates \
    gnupg \
    lsb-release \
    terminator \
    tmux \
    screen \
    rsync \
    zip \
    unzip \
    neofetch

# Install additional useful tools
apt-get install -y \
    can-utils \
    i2c-tools \
    usbutils \
    pciutils

echo "✓ Development tools installed"

# Step 6: Install ROS 2 Humble (for Ubuntu 22.04)
echo -e "${GREEN}[6/7] Installing ROS 2 Humble...${NC}"

# Set locale
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 apt repository
apt-get install -y software-properties-common
add-apt-repository universe
apt-get update && apt-get install -y curl gnupg lsb-release

# Add ROS 2 GPG key (modern method for Ubuntu 22.04+)
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS 2 repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2-latest.list > /dev/null

# Update and install ROS 2
apt-get update
apt-get install -y ros-humble-desktop

# Install ROS 2 development tools
apt-get install -y \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-argcomplete

# Initialize rosdep
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    rosdep init
fi
rosdep update

# Source ROS 2 setup
echo "source /opt/ros/humble/setup.bash" >> /home/nano/.bashrc

echo "✓ ROS 2 Humble installed"

# Step 7: Final configuration
echo -e "${GREEN}[7/7] Final configuration...${NC}"

# Ensure SSH is enabled
systemctl enable ssh
systemctl start ssh

# Set up bash completion for ROS
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/nano/.bashrc

# Create ROS workspace directory
mkdir -p /home/nano/ros2_ws/src
chown -R nano:nano /home/nano/ros2_ws

echo "✓ Configuration complete"

echo ""
echo -e "${GREEN}=========================================="
echo "Setup Complete!"
echo "==========================================${NC}"
echo ""
echo "Next steps:"
echo "1. Reboot the system: sudo reboot"
echo "2. After reboot, verify hostname: hostname (should show 'isaac')"
echo "3. Verify mDNS: avahi-browse -a (should show isaac.local)"
echo "4. Test ROS 2: source /opt/ros/humble/setup.bash && ros2 --help"
echo ""
echo "To connect from another computer:"
echo "  ssh nano@isaac.local"
echo "  or"
echo "  ssh nano@<current-ip-address>"
echo ""
echo "Current IP address:"
ip addr show | grep "inet " | grep -v 127.0.0.1 | awk '{print $2}' | cut -d/ -f1

