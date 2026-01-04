# Quick Start Guide - Isaac Jetson Orin Nano

## First-Time Setup

### Step 1: Initial System Setup (Jetson Only)

After flashing the Jetson Orin Nano:

```bash
cd ~/src/jetson-orin-nano
sudo ./scripts/system/setup_isaac.sh
sudo reboot
```

### Step 2: Unified Setup (All Systems)

After initial setup (or on any system):

```bash
cd ~/src/jetson-orin-nano
./setup.sh
```

### Step 3: Activate Environment

```bash
source scripts/utils/env_setup.sh
```

## Docker Quick Start

```bash
# Build and run
docker-compose build
docker-compose run --rm isaac-dev

# Inside container
./setup.sh
source scripts/utils/env_setup.sh
```

## Verify Setup

```bash
# Check hostname
hostname  # Should show "isaac"

# Check ROS 2
source /opt/ros/humble/setup.bash
ros2 --help

# Run system monitor
ros2 launch system_monitor system_monitor.launch.py
```

## Key Information

- **Hostname**: isaac
- **Username**: nano
- **Network**: Dynamic IP (DHCP) + mDNS hostname resolution
- **Access**: `ssh nano@isaac.local`
- **ROS Version**: Humble (ROS 2)
- **Workspace**: ~/ros2_ws

## Useful Commands

```bash
# Find current IP address
hostname -I
ip addr show

# Check mDNS status
sudo systemctl status avahi-daemon

# Check ROS 2 installation
ros2 doctor

# Create new ROS 2 package
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake my_package
```

## Documentation

- **[WORKFLOW.md](WORKFLOW.md)** - Complete development workflow
- **[SETUP.md](docs/setup/SETUP.md)** - Detailed setup instructions
- **[BRINGUP.md](BRINGUP.md)** - Comprehensive bringup documentation
