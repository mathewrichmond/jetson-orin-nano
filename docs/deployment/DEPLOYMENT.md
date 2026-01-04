# Deployment Guide

## Overview

The Isaac robot system supports multiple deployment modes:
1. **Dev Sandbox** - Development environment with live code changes
2. **Installed Package** - Production installation from released packages
3. **Dev Priority** - Dev sandbox takes precedence over installed package

## Deployment Modes

### Dev Sandbox Deployment

The dev sandbox is located at `/home/nano/src/jetson-orin-nano` and takes precedence over installed packages.

**Quick Deploy**:
```bash
./scripts/deployment/deploy_dev.sh
```

This will:
- Create archive of current code
- Deploy to target dev sandbox
- Optionally restart services

**Manual Deploy**:
```bash
# From development machine
rsync -avz --exclude='.git' --exclude='build' --exclude='install' \
    ./ nano@isaac.local:/home/nano/src/jetson-orin-nano/
```

### Package Installation

Install from a released package:

```bash
# Download package from GitHub releases
wget https://github.com/user/repo/releases/download/v1.0.0/isaac-robot-1.0.0.tar.gz

# Install package
sudo ./scripts/packaging/install_package.sh isaac-robot-1.0.0.tar.gz

# Run setup
isaac-setup
```

### Package Priority

The system checks for Isaac root in this order:
1. Dev sandbox: `/home/nano/src/jetson-orin-nano` (if exists)
2. Installed package: `/opt/isaac-robot`
3. Environment variables: `ISAAC_DEV_DIR` or `ISAAC_INSTALL_DIR`

Use `scripts/utils/find_isaac_root.py` to find the active root:
```bash
python3 scripts/utils/find_isaac_root.py
```

## Auto-Start on Boot

Systemd services are provided for auto-start:

### Install Services

```bash
sudo ./scripts/system/install_services.sh
```

This installs:
- `isaac-system-monitor.service` - System monitoring
- `isaac-robot.service` - Main robot system

### Service Management

```bash
# Start services
sudo systemctl start isaac-system-monitor
sudo systemctl start isaac-robot

# Stop services
sudo systemctl stop isaac-system-monitor
sudo systemctl stop isaac-robot

# Check status
sudo systemctl status isaac-system-monitor
sudo systemctl status isaac-robot

# View logs
sudo journalctl -u isaac-system-monitor -f
sudo journalctl -u isaac-robot -f
```

## GitHub CI/CD

### Continuous Integration

GitHub Actions automatically:
- Lint and format check on PR
- Run tests
- Build packages
- Test Docker builds

### Release Workflow

When a release is created:
1. Builds release package
2. Creates checksums
3. Uploads to GitHub releases
4. Optionally deploys to target

### Deployment Secrets

Configure GitHub secrets:
- `DEPLOY_HOST` - Target hostname/IP
- `DEPLOY_USER` - SSH username
- `DEPLOY_SSH_KEY` - SSH private key

## Building Packages

### Build Package Locally

```bash
./scripts/packaging/build_package.sh [version]
```

Creates package in `dist/` directory:
- `isaac-robot-<version>.tar.gz`
- `isaac-robot-<version>.tar.gz.sha256`

### Package Contents

Packages include:
- All source code
- Configuration files
- Scripts
- Documentation
- Installation script

## Deployment Workflow

### Development Workflow

1. **Make changes** in dev sandbox
2. **Test locally** or in Docker
3. **Deploy to target**: `./scripts/deployment/deploy_dev.sh`
4. **Services auto-restart** (if configured)

### Release Workflow

1. **Create release** on GitHub
2. **CI builds package** automatically
3. **Download package** from releases
4. **Install on target**: `sudo ./scripts/packaging/install_package.sh <package>`
5. **Run setup**: `isaac-setup`

## Troubleshooting

### Dev Sandbox Not Found

Check if dev sandbox exists:
```bash
ls -la /home/nano/src/jetson-orin-nano/setup.sh
```

If missing, deploy dev sandbox:
```bash
./scripts/deployment/deploy_dev.sh
```

### Services Not Starting

Check service status:
```bash
sudo systemctl status isaac-robot
sudo journalctl -u isaac-robot -n 50
```

Check if ROS 2 is available:
```bash
source /opt/ros/humble/setup.bash
ros2 --help
```

### Package Priority Issues

Find active root:
```bash
python3 scripts/utils/find_isaac_root.py
```

Check environment variables:
```bash
echo $ISAAC_DEV_DIR
echo $ISAAC_INSTALL_DIR
```

## Best Practices

1. **Use dev sandbox** for active development
2. **Use installed packages** for production
3. **Test in Docker** before deploying
4. **Version packages** properly
5. **Monitor services** after deployment
6. **Keep dev sandbox** in sync with git
