# System Setup Guide

## Overview

The Isaac robot system uses a **unified setup workflow** that works across all environments. This guide covers both initial system setup and development environment setup.

## Quick Start

### 1. Initial Jetson Setup (First Time Only)

After flashing the Jetson Orin Nano, run the initial system setup:

```bash
cd ~/src/jetson-orin-nano
sudo ./scripts/system/setup_isaac.sh
sudo reboot
```

This sets up:
- Hostname (`isaac`)
- mDNS (hostname resolution)
- Network configuration
- Basic system packages
- ROS 2 Humble

### 2. Unified Setup (All Environments)

After the initial setup (or on any system), use the unified setup:

```bash
cd ~/src/jetson-orin-nano
./setup.sh
```

This handles:
- System package installation
- Python package installation
- ROS 2 workspace setup
- Virtual environment creation
- Pre-commit hooks

### 3. Activate Environment

```bash
source scripts/utils/env_setup.sh
```

## Detailed Setup

### Initial Jetson Setup

The `scripts/system/setup_isaac.sh` script performs:

1. **Hostname Configuration**: Sets hostname to `isaac`
2. **mDNS Setup**: Configures Avahi for `.local` hostname resolution
3. **Network Configuration**: Ensures DHCP is enabled
4. **System Updates**: Updates all system packages
5. **Development Tools**: Installs basic development tools
6. **ROS 2 Installation**: Installs ROS 2 Humble
7. **Final Configuration**: Sets up SSH, ROS workspace, etc.

**Note**: This script should only be run once after flashing the Jetson.

### Unified Setup

The `setup.sh` script is the main entry point for all setup operations:

- **Idempotent**: Safe to run multiple times
- **Environment-aware**: Detects Jetson/Docker/Ubuntu
- **Configuration-driven**: Uses YAML configs for packages
- **State tracking**: Remembers completed steps

See [WORKFLOW.md](../../WORKFLOW.md) for detailed workflow documentation.

## Docker Setup

For Docker environments:

```bash
# Build container
docker-compose build

# Run container
docker-compose run --rm isaac-dev

# Inside container, run setup
./setup.sh
```

## Verification

After setup, verify installation:

```bash
# Check hostname
hostname  # Should show "isaac"

# Check ROS 2
source /opt/ros/humble/setup.bash
ros2 --help

# Check system health
./scripts/monitoring/system_health_check.sh
```

## Troubleshooting

### Setup Fails

1. Check setup log: `cat .setup.log`
2. Check setup state: `cat .setup_state`
3. Reset and retry: `rm .setup_state && ./setup.sh`

### ROS 2 Not Found

- On Jetson: Ensure `scripts/system/setup_isaac.sh` was run first
- In Docker: Use `docker-compose run isaac-ros` (has ROS 2 pre-installed)
- On Ubuntu: Install ROS 2 manually or use Docker

### Package Installation Fails

1. Update package list: `sudo apt-get update`
2. Check configuration: `python3 scripts/utils/package_manager.py list system`
3. Try dry-run: `python3 scripts/utils/package_manager.py install-system --groups dev_minimal --dry-run`

## Next Steps

- See [WORKFLOW.md](../../WORKFLOW.md) for development workflow
- See [DEVELOPMENT_ENVIRONMENT.md](../development/DEVELOPMENT_ENVIRONMENT.md) for development setup
- See [PACKAGE_MANAGEMENT.md](../development/PACKAGE_MANAGEMENT.md) for package management
