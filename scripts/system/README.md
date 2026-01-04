# System Setup Scripts

This directory contains system-level setup scripts.

## Scripts

### `setup_isaac.sh`

**Purpose**: Initial Jetson system setup (run once after flashing)

**When to use**: First time setup after flashing the Jetson Orin Nano

**What it does**:
- Sets hostname to `isaac`
- Configures mDNS (Avahi) for hostname resolution
- Sets up network configuration
- Installs basic system packages
- Installs ROS 2 Humble

**Usage**:
```bash
sudo ./scripts/system/setup_isaac.sh
sudo reboot
```

**Note**: This should only be run once. After this, use the unified `setup.sh` script.

## Unified Setup

For all development environment setup, use the main setup script:

```bash
cd ~/src/jetson-orin-nano
./setup.sh
```

See [Development Workflow](../../docs/development/WORKFLOW.md) for complete workflow documentation.
