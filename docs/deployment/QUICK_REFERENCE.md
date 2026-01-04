# Deployment Quick Reference

## Quick Deploy Dev Changes

```bash
./scripts/deployment/deploy_dev.sh
```

## Install from Package

```bash
# Download from GitHub releases
wget <release-url>/isaac-robot-<version>.tar.gz

# Install
sudo ./scripts/packaging/install_package.sh isaac-robot-<version>.tar.gz

# Setup
isaac-setup
```

## Build Package Locally

```bash
./scripts/packaging/build_package.sh [version]
```

## Install Auto-Start Services

```bash
sudo ./scripts/system/install_services.sh
```

## Service Management

```bash
# Start
sudo systemctl start isaac-system-monitor
sudo systemctl start isaac-robot

# Stop
sudo systemctl stop isaac-system-monitor
sudo systemctl stop isaac-robot

# Status
sudo systemctl status isaac-system-monitor
sudo systemctl status isaac-robot

# Logs
sudo journalctl -u isaac-system-monitor -f
sudo journalctl -u isaac-robot -f
```

## Find Active Root

```bash
python3 scripts/utils/find_isaac_root.py
```

## Package Priority

1. Dev sandbox: `/home/nano/src/jetson-orin-nano`
2. Installed: `/opt/isaac-robot`
3. Environment variables: `ISAAC_DEV_DIR` or `ISAAC_INSTALL_DIR`
