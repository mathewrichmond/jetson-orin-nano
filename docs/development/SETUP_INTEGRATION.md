# Setup Integration Guide

## Overview

The Isaac robot system uses a **unified setup workflow** that works identically across all environments:
- Native Jetson hardware
- Docker containers
- Ubuntu development machines

## Key Components

### 1. Main Setup Script (`setup.sh`)

Single entry point for all setup operations:
- **Idempotent**: Safe to run multiple times
- **Environment-aware**: Detects Jetson/Docker/Ubuntu
- **State tracking**: Remembers completed steps
- **Comprehensive**: Handles all dependencies

### 2. Docker Support

- `Dockerfile` - Base development image
- `Dockerfile.ros2` - Image with ROS 2 pre-installed
- `docker-compose.yml` - Multi-service configuration
- `.dockerignore` - Excludes unnecessary files

### 3. Environment Setup (`scripts/utils/env_setup.sh`)

Activates development environment:
- Python virtual environment
- ROS 2 setup
- ROS 2 workspace
- PATH and PYTHONPATH configuration

### 4. Package Management

- Configuration-driven (`config/system/packages.yaml`)
- Unified package manager (`scripts/utils/package_manager.py`)
- Environment-specific package selection

## Workflow

### Standard Workflow

```bash
# 1. Clone repository
git clone <repo>
cd jetson-orin-nano

# 2. Run setup (one command)
./setup.sh

# 3. Activate environment
source scripts/utils/env_setup.sh

# 4. Run code
ros2 launch system_monitor system_monitor.launch.py
```

### Docker Workflow

```bash
# 1. Build and run
docker-compose build
docker-compose run --rm isaac-dev

# 2. Inside container (same workflow)
./setup.sh
source scripts/utils/env_setup.sh
```

## Setup Steps

The setup script performs these steps (in order):

1. **Update System** - Update package lists
2. **Install System Packages** - Install apt packages from config
3. **Install Python Packages** - Install pip packages from config
4. **Setup ROS 2 Workspace** - Initialize and build workspace
5. **Setup Virtual Environment** - Create Python venv
6. **Install Pre-commit Hooks** - Set up git hooks

Each step is tracked in `.setup_state` and skipped if already completed.

## Environment Detection

The setup script automatically detects:

- **Jetson**: Checks for `/etc/nv_tegra_release`
  - Installs: `robot_essentials` package group

- **Docker**: Checks for `/.dockerenv` or docker cgroup
  - Installs: `dev_minimal` package group

- **Ubuntu**: Checks `/etc/os-release`
  - Installs: `dev_full` package group

## State Management

### Setup State File

Location: `.setup_state`

Tracks completed steps:
```
update_system
install_system_packages
install_python_packages
setup_ros2_workspace
setup_venv
install_precommit
```

### Resetting Setup

To re-run setup from scratch:
```bash
rm .setup_state
./setup.sh
```

### Setup Log

Location: `.setup.log`

Contains detailed log of all setup operations.

## Integration Points

### Package Configuration

All packages defined in:
- `config/system/packages.yaml` - System packages
- `config/system/python_packages.yaml` - Python packages

### Scripts Integration

Setup script calls:
- `scripts/utils/package_manager.py` - Package installation
- `scripts/system/setup_isaac.sh` - Initial Jetson setup (first boot only)

### ROS 2 Integration

- Workspace: `~/ros2_ws`
- Packages linked from: `src/system_monitor`
- Build: `colcon build --symlink-install`

## Makefile Shortcuts

Convenient shortcuts via Makefile:

```bash
make setup              # Run setup
make setup-docker       # Build and setup Docker
make build-docker       # Build Docker image
make run-docker         # Run Docker container
make env                # Activate environment
make build-ros          # Build ROS 2 workspace
make clean              # Clean build artifacts
make reset              # Reset setup state
```

## Best Practices

1. **Always run setup after pulling** - Dependencies may change
2. **Use virtual environment** - Isolates dependencies
3. **Commit configuration changes** - Track dependency changes
4. **Use Docker for testing** - Ensures consistency
5. **Check setup log** - Debug issues with `.setup.log`

## Troubleshooting

### Setup Fails

1. Check log: `cat .setup.log`
2. Check state: `cat .setup_state`
3. Reset: `rm .setup_state && ./setup.sh`

### Environment Not Detected

Manually set environment:
```bash
export ENV_TYPE=jetson  # or docker, ubuntu
./setup.sh
```

### Package Installation Fails

1. Update packages: `sudo apt-get update`
2. Check config: `python3 scripts/utils/package_manager.py list system`
3. Dry run: `python3 scripts/utils/package_manager.py install-system --groups dev_minimal --dry-run`

## Future Enhancements

- [ ] Parallel package installation
- [ ] Setup validation checks
- [ ] Rollback capability
- [ ] Setup profiles (minimal, full, custom)
- [ ] Remote setup via SSH
- [ ] Setup caching for faster re-runs
