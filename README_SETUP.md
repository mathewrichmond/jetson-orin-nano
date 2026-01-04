# Setup Guide - Single Entry Point

## Unified Setup Script

The Isaac robot system uses **one unified setup script** that handles everything:

```bash
./setup.sh
```

## Usage

### Interactive Setup (Recommended)
```bash
./setup.sh
```
Prompts you for optional components (Bluetooth, WiFi, USB-C, services).

### Non-Interactive Setup (Auto-Yes + Auto-Reboot)
```bash
NON_INTERACTIVE=true sudo ./setup.sh
```
Auto-answers 'yes' to all prompts and reboots if needed.

## What It Does

The setup script handles all configuration:

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

## After Setup

The script automatically checks if a reboot is needed and will:
- **Interactive mode**: Ask if you want to reboot
- **Non-interactive mode**: Reboot automatically after 10 seconds

## Check Setup Status

```bash
# See completed steps
cat .setup_state

# See setup log
tail -50 .setup.log
```

## Re-run Setup

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

## Reset Setup

To start fresh:
```bash
rm .setup_state .setup.log
./setup.sh
```

## Environment Detection

The script automatically detects:
- **Jetson** - Full setup with hardware support
- **Docker** - Minimal setup for containers
- **Ubuntu** - Full development environment

## Quick Reference

```bash
# Run setup
./setup.sh

# Run with auto-yes and auto-reboot
NON_INTERACTIVE=true sudo ./setup.sh

# Check status
cat .setup_state

# View log
tail -f .setup.log
```
