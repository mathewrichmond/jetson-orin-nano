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
- Optional components (Bluetooth, WiFi, USB-C, services)

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

#### Usage

**Interactive Setup** (Recommended):
```bash
./setup.sh
```
Prompts you for optional components (Bluetooth, WiFi, USB-C, services).

**Non-Interactive Setup** (Auto-Yes + Auto-Reboot):
```bash
NON_INTERACTIVE=true sudo ./setup.sh
```
Auto-answers 'yes' to all prompts and reboots if needed.

#### What It Does

1. **System Updates** - Updates package lists
2. **System Packages** - Installs packages from `config/system/packages.yaml`
3. **Python Packages** - Installs packages from `config/system/python_packages.yaml`
4. **ROS 2 Workspace** - Sets up and builds ROS 2 workspace
5. **Virtual Environment** - Creates Python venv
6. **Pre-commit Hooks** - Installs git hooks
7. **Bluetooth** (optional) - Sets up Bluetooth support
8. **WiFi** (optional) - Configures WiFi with Ethernet priority
9. **USB-C Display** (optional) - Sets up USB-C display/dock support
10. **Systemd Services** (optional) - Installs auto-start services

#### After Setup

The script automatically checks if a reboot is needed and will:
- **Interactive mode**: Ask if you want to reboot
- **Non-interactive mode**: Reboot automatically after 10 seconds

#### Check Setup Status

```bash
# See completed steps
cat .setup_state

# See setup log
tail -50 .setup.log
```

#### Re-run Setup

The setup is **idempotent** - safe to run multiple times:
- Already completed steps are skipped
- Only new/missing components are installed

To re-run a specific step:
```bash
# Remove step from state
sed -i '/step_name/d' .setup_state

# Re-run setup
./setup.sh
```

#### Reset Setup

To start fresh:
```bash
rm .setup_state .setup.log
./setup.sh
```

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

- See [Development Workflow](../development/WORKFLOW.md) for development workflow
- See [Development Environment](../development/DEVELOPMENT_ENVIRONMENT.md) for development setup
- See [Package Management](../development/PACKAGE_MANAGEMENT.md) for package management
