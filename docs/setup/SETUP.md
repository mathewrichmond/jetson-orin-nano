# System Setup Guide

## Initial System Setup

### Prerequisites
- Jetson Orin Nano flashed with JetPack 5.x
- Network connection (Ethernet or WiFi)
- SSH access (optional but recommended)

### First Boot Setup

1. **Run the automated setup script**:
   ```bash
   cd ~/src/jetson-orin-nano
   sudo ./scripts/system/setup_isaac.sh
   sudo reboot
   ```

2. **After reboot, verify setup**:
   ```bash
   hostname  # Should show "isaac"
   source /opt/ros/humble/setup.bash
   ros2 --help
   ```

### Manual Setup Steps

If you need to customize the setup, see the individual scripts in `scripts/system/`:
- `setup_isaac.sh` - Main system setup
- Additional scripts for specific components

## Hardware Setup

### Realsense Cameras

See `docs/hardware/realsense.md` for detailed setup instructions.

### Motor Controllers

See `docs/hardware/motor_controllers.md` for setup and configuration.

### Raspberry Pi Sub-modules

See `docs/hardware/raspberry_pi_modules.md` for integration guide.

## ROS 2 Workspace Setup

The system setup script creates a ROS 2 workspace at `~/ros2_ws`. To use it:

```bash
cd ~/ros2_ws/src
# Clone or create your ROS 2 packages here
colcon build
source ~/ros2_ws/install/setup.bash
```

## Configuration

System configurations are stored in `config/`:
- `config/system/` - System-wide settings
- `config/hardware/` - Hardware-specific configs
- `config/control/` - Control mode configurations

## Network Configuration

- **Hostname**: `isaac`
- **Access**: `ssh nano@isaac.local` (via mDNS)
- **Dynamic IP**: Configured via NetworkManager/DHCP

## Troubleshooting

See `BRINGUP.md` in the root directory for detailed troubleshooting steps.

## Next Steps

1. Set up hardware components (see `docs/hardware/`)
2. Configure control modes (see `docs/architecture/ARCHITECTURE.md`)
3. Set up monitoring (see `scripts/monitoring/`)
4. Configure logging (see `logging/config/`)

