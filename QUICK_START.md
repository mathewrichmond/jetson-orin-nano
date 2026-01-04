# Quick Start Guide - Isaac Jetson Orin Nano

## First Boot Setup

After flashing and first boot, run the automated setup:

```bash
cd ~/Documents
sudo ./setup_isaac.sh
sudo reboot
```

## After Reboot - Verify Setup

```bash
# Check hostname
hostname  # Should show "isaac"

# Check network (should show dynamic IP)
ip addr show

# Check mDNS
avahi-browse -a  # Should show isaac.local

# Test ROS 2
source /opt/ros/humble/setup.bash
ros2 --help
```

## Connect from Another Computer

```bash
# Using hostname (recommended)
ssh nano@isaac.local

# Or find current IP first
ssh nano@isaac.local "hostname -I"
```

## Key Information

- **Hostname**: isaac
- **Username**: nano
- **Password**: nano
- **Network**: Dynamic IP (DHCP) + mDNS hostname resolution
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

See `BRINGUP.md` for comprehensive setup documentation and troubleshooting.

