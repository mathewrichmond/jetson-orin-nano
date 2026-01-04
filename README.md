# Jetson Orin Nano System Bringup

This repository contains documentation and setup scripts for bringing up a Jetson Orin Nano system named "Isaac".

## Quick Start

After flashing and first boot:

```bash
cd ~/src/jetson-orin-nano
sudo ./setup_isaac.sh
sudo reboot
```

## Contents

- **`setup_isaac.sh`** - Automated setup script for first-time system bringup
- **`BRINGUP.md`** - Comprehensive bringup documentation
- **`QUICK_START.md`** - Quick reference guide

## Features

- Dynamic IP configuration (DHCP) with mDNS hostname resolution
- System updates and security patches
- Development tools installation
- ROS 2 Humble installation and configuration
- SSH setup and configuration

## Network Configuration

The system is configured to use:
- **Hostname**: `isaac`
- **Network**: Dynamic IP (DHCP) via NetworkManager
- **Hostname Resolution**: mDNS (Avahi) - accessible as `isaac.local`

## Documentation

See `BRINGUP.md` for detailed setup instructions and troubleshooting.

## Author

Mathew Richmond - mathewrichmond@gmail.com

