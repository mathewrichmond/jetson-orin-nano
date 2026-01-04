# Isaac Robot System - Development Workflow

## Overview

This document describes the simple, consistent workflow for setting up and running the Isaac robot system. The workflow works identically whether you're running on the native Jetson hardware or in a Docker container.

## Quick Start

### 1. Clone the Repository

```bash
git clone <repository-url>
cd jetson-orin-nano
```

### 2. Run Setup

```bash
./setup.sh
```

That's it! The setup script will:
- Detect your environment (Jetson, Docker, or Ubuntu)
- Install all required packages
- Set up ROS 2 workspace
- Create Python virtual environment
- Install pre-commit hooks

### 3. Activate Environment

```bash
source scripts/utils/env_setup.sh
```

Or manually:
```bash
source .venv/bin/activate
source /opt/ros/humble/setup.bash  # If ROS 2 is available
source ~/ros2_ws/install/setup.bash  # If workspace is built
```

### 4. Run Code

```bash
# Run system monitor
ros2 launch system_monitor system_monitor.launch.py

# Or run any other ROS 2 nodes
ros2 run <package> <node>
```

## Docker Workflow

### Build and Run Container

```bash
# Build container
docker-compose build

# Run container
docker-compose run --rm isaac-dev

# Or with ROS 2 pre-installed
docker-compose run --rm isaac-ros
```

### Inside Container

The setup is identical:
```bash
./setup.sh
source scripts/utils/env_setup.sh
```

## Setup Script Features

### Idempotent

The setup script is **idempotent** - you can run it multiple times safely. It tracks completed steps and skips them on subsequent runs.

### Environment Detection

Automatically detects:
- **Jetson**: Installs robot essentials
- **Docker**: Installs minimal dev environment
- **Ubuntu**: Installs full development environment

### State Tracking

Setup state is stored in `.setup_state`. To reset:
```bash
rm .setup_state
./setup.sh
```

## Development Workflow

### Daily Development

1. **Pull latest changes**
   ```bash
   git pull
   ```

2. **Run setup** (if dependencies changed)
   ```bash
   ./setup.sh
   ```

3. **Activate environment**
   ```bash
   source scripts/utils/env_setup.sh
   ```

4. **Develop and test**
   ```bash
   # Make changes, test, etc.
   ```

### Adding Dependencies

1. **Edit configuration files**
   - System packages: `config/system/packages.yaml`
   - Python packages: `config/system/python_packages.yaml`

2. **Run setup** (will install new packages)
   ```bash
   ./setup.sh
   ```

3. **Commit changes**
   ```bash
   git add config/system/*.yaml
   git commit -m "Add new dependencies"
   ```

### Building ROS 2 Packages

```bash
# Activate environment
source scripts/utils/env_setup.sh

# Build workspace
cd ~/ros2_ws
colcon build --symlink-install

# Source workspace
source install/setup.bash
```

## Project Structure

```
jetson-orin-nano/
├── setup.sh                 # Main setup script (run this first)
├── Dockerfile               # Docker image for development
├── docker-compose.yml       # Docker Compose configuration
├── config/                  # Configuration files
│   └── system/             # System package configurations
├── scripts/                 # Utility scripts
│   ├── system/             # System setup scripts
│   ├── utils/              # Utility scripts
│   └── ...
├── src/                     # Source code
│   └── system_monitor/      # ROS 2 packages
└── .venv/                   # Python virtual environment (created by setup)
```

## Environment Variables

- `SETUP_LOG`: Path to setup log file (default: `.setup.log`)
- `SETUP_STATE`: Path to setup state file (default: `.setup_state`)
- `ISAAC_PROJECT_ROOT`: Project root directory (set by env_setup.sh)
- `PYTHONPATH`: Includes project root (set by env_setup.sh)

## Troubleshooting

### Setup Fails

1. Check setup log: `cat .setup.log`
2. Check setup state: `cat .setup_state`
3. Reset and retry: `rm .setup_state && ./setup.sh`

### ROS 2 Not Found

- On Jetson: Run `scripts/system/setup_isaac.sh` first
- In Docker: Use `docker-compose run isaac-ros` (has ROS 2 pre-installed)
- On Ubuntu: Install ROS 2 manually or use Docker

### Package Installation Fails

1. Update package list: `sudo apt-get update`
2. Check configuration: `python3 scripts/utils/package_manager.py list system`
3. Try dry-run: `python3 scripts/utils/package_manager.py install-system --groups dev_minimal --dry-run`

### Docker Issues

1. Rebuild container: `docker-compose build --no-cache`
2. Check Dockerfile: Ensure all dependencies are listed
3. Check logs: `docker-compose logs`

## Best Practices

1. **Always run setup after pulling changes** - Dependencies may have changed
2. **Use virtual environment** - Keeps dependencies isolated
3. **Commit configuration changes** - Track dependency changes in git
4. **Use Docker for testing** - Ensures consistent environment
5. **Check setup log** - If something fails, check `.setup.log`

## Advanced Usage

### Custom Setup Steps

Add custom steps to `setup.sh` or create new scripts in `scripts/system/`.

### Multiple Environments

Use different Docker Compose files:
```bash
docker-compose -f docker-compose.dev.yml up
```

### CI/CD Integration

The setup script can be used in CI/CD pipelines:
```bash
./setup.sh
source scripts/utils/env_setup.sh
# Run tests, build, etc.
```

## Next Steps

- See [DEVELOPMENT_ENVIRONMENT.md](docs/development/DEVELOPMENT_ENVIRONMENT.md) for detailed development setup
- See [PACKAGE_MANAGEMENT.md](docs/development/PACKAGE_MANAGEMENT.md) for package management
- See [SYSTEM_MONITOR.md](docs/monitoring/SYSTEM_MONITOR.md) for system monitoring

