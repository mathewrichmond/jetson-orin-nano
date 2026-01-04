# **Setting up Isaac**

## *Jetson Orin Nano*

[Mathew Richmond](mailto:mathewrichmond@gmail.com)Jan 4, 2026

Username: nano  
Password: nano

**Setup**  
[Setup Guide](https://forums.developer.nvidia.com/t/how-to-get-into-recovery-mode/250525)  
[Quick Start](https://docs.nvidia.com/jetson/archives/r35.2.1/DeveloperGuide/text/IN/QuickStart.html#jetson-modules-and-configurations)  
[Getting Started Guide](https://developer.nvidia.com/embedded/learn/get-started-jetson-orin-nano-devkit#prepare)  
[Recovery Mode](https://forums.developer.nvidia.com/t/how-to-get-into-recovery-mode/250525)

**Bringup**  
[LeRobot](https://www.jetson-ai-lab.com/archive/lerobot.html)

## **First-Time System Bringup (After Flashing)**

This section covers the comprehensive system bringup after first boot, including network configuration with dynamic IP and hostname resolution, system updates, and ROS installation.

### **Quick Start: Automated Setup Script**

The easiest way to perform the complete bringup is to run the automated setup script:

```bash
cd ~/Documents
sudo ./setup_isaac.sh
```

This script will:
1. Set hostname to `isaac`
2. Configure mDNS (Avahi) for hostname resolution
3. Verify NetworkManager is using DHCP (dynamic IP)
4. Update all system packages
5. Install development tools
6. Install ROS 2 Humble
7. Configure SSH and create ROS workspace

**After running the script, reboot:**
```bash
sudo reboot
```

### **Manual Setup (Step-by-Step)**

If you prefer to run the steps manually or need to customize the setup:

#### **Step 1: Network Configuration (Dynamic IP + Hostname Resolution)**

**1.1 Set Hostname**
```bash
sudo hostnamectl set-hostname isaac
sudo sed -i 's/127.0.1.1\tubuntu/127.0.1.1\tisaac isaac.local/' /etc/hosts
echo "127.0.0.1 isaac isaac.local" | sudo tee -a /etc/hosts
```

**1.2 Configure mDNS (Avahi) for Hostname Resolution**

Avahi is already installed. Enable workstation publishing:
```bash
sudo sed -i 's/#publish-workstation=no/publish-workstation=yes/' /etc/avahi/avahi-daemon.conf
sudo systemctl restart avahi-daemon
```

**1.3 Verify NetworkManager is Using DHCP**

Check current network status:
```bash
nmcli connection show
ip addr show
```

The network should show `dynamic` in the IP address output. If you need to ensure DHCP is enabled:
```bash
# Get active connection name
CONN_NAME=$(nmcli -t -f NAME connection show --active | head -1)
# Ensure DHCP is enabled
sudo nmcli connection modify "$CONN_NAME" ipv4.method auto
sudo nmcli connection up "$CONN_NAME"
```

**1.4 Verify mDNS is Working**

After reboot, test hostname resolution:
```bash
# On Isaac itself
avahi-browse -a
hostname  # Should show "isaac"

# From another computer on the network
ping isaac.local
ssh nano@isaac.local
```

#### **Step 2: System Updates**

```bash
sudo apt-get update
sudo apt-get upgrade -y
sudo apt-get dist-upgrade -y
sudo apt-get autoremove -y
sudo apt-get autoclean
```

#### **Step 3: Install Development Tools**

```bash
sudo apt-get install -y \
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
    neofetch \
    can-utils \
    i2c-tools \
    usbutils \
    pciutils
```

#### **Step 4: Install ROS 2 Humble**

ROS 2 Humble is the recommended version for Ubuntu 22.04 (Jammy).

**4.1 Set Locale**
```bash
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

**4.2 Add ROS 2 Repository**
```bash
sudo apt-get install -y software-properties-common
sudo add-apt-repository universe
sudo apt-get update && sudo apt-get install -y curl gnupg lsb-release

# Add ROS 2 GPG key (modern method for Ubuntu 22.04+)
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS 2 repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2-latest.list > /dev/null
```

**4.3 Install ROS 2**
```bash
sudo apt-get update
sudo apt-get install -y ros-humble-desktop

# Install development tools
sudo apt-get install -y \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-argcomplete

# Initialize rosdep
sudo rosdep init
rosdep update
```

**4.4 Configure ROS 2 Environment**

Add to `~/.bashrc`:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
source ~/.bashrc
```

**4.5 Create ROS Workspace**
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

#### **Step 5: Verify Installation**

After completing setup and rebooting:

```bash
# Verify hostname
hostname  # Should show "isaac"

# Verify network (should show dynamic IP)
ip addr show

# Verify mDNS
avahi-browse -a  # Should show isaac.local

# Verify ROS 2
source /opt/ros/humble/setup.bash
ros2 --help  # Should show ROS 2 help

# Check ROS 2 version
ros2 doctor
```

### **Post-Setup Verification Checklist**

- [ ] Hostname is set to `isaac`
- [ ] Network is using DHCP (dynamic IP)
- [ ] mDNS is working (`ping isaac.local` works from other computers)
- [ ] System packages are up to date
- [ ] Development tools are installed
- [ ] ROS 2 Humble is installed and working
- [ ] SSH is enabled and accessible
- [ ] ROS workspace created at `~/ros2_ws`

### **Network Configuration Details**

**Dynamic IP (DHCP):**
- NetworkManager automatically obtains IP address from router
- IP address may change on reboot (this is expected)
- Hostname resolution via mDNS allows access via `isaac.local` regardless of IP

**mDNS (Multicast DNS):**
- Avahi daemon provides `.local` hostname resolution
- Works automatically on local network
- No router configuration needed
- Accessible as `isaac.local` from any device on the network

**Benefits of Dynamic IP + mDNS:**
- No manual IP configuration needed
- Works with any router/DHCP server
- Hostname remains constant even if IP changes
- Easy to add to multiple computers without IP conflicts

## **Initial Setup (First Time Only)**

### **1\. Add SSH Key to Isaac**

Run this command and enter the password `nano` when prompted:

cat \~/.ssh/id\_rsa.pub | ssh nano@192.168.0.129 "mkdir \-p \~/.ssh && chmod 700 \~/.ssh && cat \>\> \~/.ssh/authorized\_keys && chmod 600 \~/.ssh/authorized\_keys"

### **2\. Set Hostname on Isaac**

SSH into Isaac and set the hostname:

ssh nano@192.168.0.129

\# Enter password: nano

\# Once connected, run:

sudo hostnamectl set-hostname isaac

echo '127.0.0.1 isaac isaac.local' | sudo tee \-a /etc/hosts

exit

### **3\. Update SSH Config**

After setting the hostname, update your SSH config to use `isaac.local` instead of the IP address. Edit `~/.ssh/config` and change:

Host isaac

    HostName 192.168.0.129

to:

Host isaac

    HostName isaac.local

## **Adding Isaac to a New Computer**

### **Step 1: Add Local Hostname Resolution (Optional but Recommended)**

To enable `ping isaac` and other name-based access from your local machine, add Isaac to your `/etc/hosts` file:

echo "192.168.0.129 isaac isaac.local" | sudo tee \-a /etc/hosts

**Note:** This only affects name resolution on this computer. For network-wide resolution, ensure mDNS/Bonjour is working or set up proper DNS.

### **Step 2: Add Entry to SSH Config**

Add the following entry to `~/.ssh/config`:

Host isaac

    HostName isaac.local

    User nano

    IdentityFile \~/.ssh/id\_rsa

**Note:** If `isaac.local` doesn't resolve, use `192.168.0.129` temporarily until DNS/mDNS is configured.

### **Step 3: Copy Your SSH Public Key**

If you don't have an SSH key, generate one:

ssh-keygen \-t rsa \-b 4096 \-C "your\_email@example.com"

Copy your public key to Isaac:

cat \~/.ssh/id\_rsa.pub | ssh nano@isaac.local "mkdir \-p \~/.ssh && chmod 700 \~/.ssh && cat \>\> \~/.ssh/authorized\_keys && chmod 600 \~/.ssh/authorized\_keys"

Or if using IP address:

cat \~/.ssh/id\_rsa.pub | ssh nano@192.168.0.129 "mkdir \-p \~/.ssh && chmod 700 \~/.ssh && cat \>\> \~/.ssh/authorized\_keys && chmod 600 \~/.ssh/authorized\_keys"

### **Step 4: Test Connection**

Test the connection:

ssh isaac

You should now be able to SSH without entering a password\!

## **Troubleshooting**

### **Network Issues**

- **Hostname not resolving**: 
  - Check if avahi-daemon is running: `sudo systemctl status avahi-daemon`
  - Restart avahi: `sudo systemctl restart avahi-daemon`
  - Verify hostname: `hostname` (should show "isaac")
  - Check `/etc/hosts` includes `isaac` and `isaac.local`
  - Temporarily use IP address: `ip addr show` to find current IP

- **mDNS not working**:
  - Ensure avahi-daemon is installed: `dpkg -l | grep avahi`
  - Check firewall: `sudo ufw status` (may need to allow mDNS)
  - Test locally: `avahi-browse -a` (should show isaac.local)
  - From another computer, ensure mDNS/Bonjour is installed

- **IP address changed after reboot**:
  - This is normal with DHCP! Use `isaac.local` instead of IP address
  - Find current IP: `ip addr show` or `hostname -I`
  - Update SSH config to use `isaac.local` instead of IP

- **Connection refused**: 
  - Verify Isaac is on the network: `ping 8.8.8.8`
  - Check SSH is running: `sudo systemctl status ssh`
  - Verify IP address: `ip addr show`
  - Check firewall: `sudo ufw status`

### **ROS Issues**

- **ROS 2 not found**: 
  - Source the setup file: `source /opt/ros/humble/setup.bash`
  - Add to `~/.bashrc` if not already there
  - Verify installation: `dpkg -l | grep ros-humble`

- **rosdep update fails**:
  - Check internet connection
  - Try: `sudo rosdep init` (may need to run again)
  - Update: `rosdep update`

### **General Issues**

- **Permission denied**: Make sure you've copied your public key correctly  
- **Package installation fails**: Run `sudo apt-get update` first
- **Hostname not persisting**: Verify `/etc/hostname` contains `isaac`
