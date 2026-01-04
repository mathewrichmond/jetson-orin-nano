# Quick Start Guide

## Initial Setup

### Single Command Setup

```bash
./setup.sh
```

That's it! The unified setup script handles everything.

### Non-Interactive Setup (Auto-Yes + Auto-Reboot)

```bash
NON_INTERACTIVE=true sudo ./setup.sh
```

## What Gets Set Up

- System packages
- Python packages
- ROS 2 workspace
- Virtual environment
- Bluetooth (optional)
- WiFi (optional)
- USB-C display (optional)
- Systemd services (optional)

## After Setup

The script will:
- Check if reboot is needed
- Prompt to reboot (or auto-reboot in non-interactive mode)
- Show next steps

## Quick Reference

### Setup
- **Full setup**: `./setup.sh`
- **Non-interactive**: `NON_INTERACTIVE=true sudo ./setup.sh`
- **Check status**: `cat .setup_state`

### Bluetooth
- **Setup**: `./scripts/system/setup_bluetooth.sh`
- **Control**: `bluetoothctl`
- **Status**: `systemctl status bluetooth`
- See [Bluetooth Setup](system/BLUETOOTH_SETUP.md) for details

### WiFi
- **Setup**: `sudo ./scripts/system/setup_wifi.sh`
- **Check**: `./scripts/system/check_network.sh`
- **Status**: `nmcli device status`
- See [WiFi Setup](system/WIFI_SETUP.md) for details

### USB-C Display
- **Check**: `./scripts/hardware/setup_usbc_display.sh`
- **Configure**: `./scripts/hardware/configure_display.sh`
- See [USB-C Display](hardware/USBC_DISPLAY.md) for details

### Robot System
- **Launch minimal**: `ros2 launch isaac_robot minimal.launch.py`
- **Launch full**: `ros2 launch isaac_robot full.launch.py`
- **Select graph**: `./scripts/utils/select_graph.sh <graph>`
- See [Robot Graph Configuration](robot/GRAPH_CONFIG.md) for details

### Deployment
- **Quick deploy**: `./scripts/deployment/quick_deploy.sh`
- **Local rebuild**: `./scripts/deployment/rebuild_local.sh`
- **Full deploy**: `./scripts/deployment/deploy_dev.sh`
- See [Deployment Guide](deployment/DEPLOYMENT.md) for details

## More Information

- **Detailed Setup**: See [Setup Guide](setup/SETUP.md)
- **Development Workflow**: See [Development Workflow](development/WORKFLOW.md)
- **First-Time Jetson Setup**: See [Bringup Guide](setup/BRINGUP.md)




