# Development Environment Setup

## Overview

The Isaac robot system uses a configuration-driven development environment. All packages, tools, and configurations are defined in YAML files, making it easy to maintain and customize.

## Quick Start

### Initial Setup

```bash
cd ~/src/jetson-orin-nano
./setup.sh
```

The unified `setup.sh` script provides:
- Configuration-driven package management
- Idempotent setup (safe to run multiple times)
- Environment detection (Jetson/Docker/Ubuntu)
- State tracking

This will:
- Install all development packages (from `config/system/packages.yaml`)
- Install Python packages (from `config/system/python_packages.yaml`)
- Set up ROS 2 workspace
- Create Python virtual environment
- Configure git and shell
- Install pre-commit hooks

### Custom Installation

Install specific package groups:

```bash
# Minimal development setup
python3 scripts/utils/package_manager.py install-system --groups dev_minimal
python3 scripts/utils/package_manager.py install-python --groups dev_minimal

# Only monitoring tools
python3 scripts/utils/package_manager.py install-system --categories monitoring
python3 scripts/utils/package_manager.py install-python --categories monitoring
```

## Configuration Files

### Package Configuration

- `config/system/packages.yaml` - System packages (apt-get)
- `config/system/python_packages.yaml` - Python packages (pip)

### Code Quality Configuration

- `.pre-commit-config.yaml` - Pre-commit hooks
- `.flake8` - Flake8 linting configuration
- `.pylintrc` - Pylint configuration
- `pyproject.toml` - Black, isort, pytest configuration
- `.editorconfig` - Editor configuration

### Development Dependencies

- `requirements-dev.txt` - Python development dependencies

## Package Management

See [PACKAGE_MANAGEMENT.md](PACKAGE_MANAGEMENT.md) for detailed documentation on:
- Adding new packages
- Using package groups
- Customizing installations

## Development Workflow

### 1. Activate Environment

```bash
source ~/venv/isaac/bin/activate
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

Or use the helper:
```bash
isaac-dev bash
```

### 2. Build ROS 2 Packages

```bash
cd ~/ros2_ws
colcon build --symlink-install
```

Or use the helper:
```bash
ros2-build
```

### 3. Run Tests

```bash
cd ~/ros2_ws
colcon test
colcon test-result --verbose
```

Or use the helper:
```bash
ros2-test
```

### 4. Code Quality Checks

```bash
# Run pre-commit hooks
pre-commit run --all-files

# Format code
black src/
isort src/

# Lint code
flake8 src/
pylint src/

# Type checking
mypy src/
```

## Adding New Packages

### System Packages

Edit `config/system/packages.yaml`:

```yaml
system:
  your_category:
    - package1
    - package2
```

### Python Packages

Edit `config/system/python_packages.yaml`:

```yaml
your_category:
  - package1>=1.0.0
  - package2>=2.0.0
```

Then install:
```bash
python3 scripts/utils/package_manager.py install-system --categories your_category
python3 scripts/utils/package_manager.py install-python --categories your_category
```

## Remote Development

### VS Code Remote SSH

1. Install VS Code Remote SSH extension
2. Connect to `isaac.local` or `nano@isaac.local`
3. Open workspace: `~/src/jetson-orin-nano`

### SSH Configuration

Add to `~/.ssh/config`:

```
Host isaac
    HostName isaac.local
    User nano
    IdentityFile ~/.ssh/id_rsa
```

## Deployment

See `scripts/deployment/deploy.sh` for deployment scripts.

```bash
# Deploy to local system
./scripts/deployment/deploy.sh

# Deploy to remote system
./scripts/deployment/deploy.sh --host isaac.local --user nano
```

## Troubleshooting

### Package Installation Fails

1. Check package exists: `apt-cache search package-name`
2. Update package list: `sudo apt-get update`
3. Check configuration: `python3 scripts/utils/package_manager.py list system`

### Python Package Issues

1. Upgrade pip: `pip3 install --upgrade pip`
2. Check Python version: `python3 --version`
3. Use virtual environment: `source ~/venv/isaac/bin/activate`

### ROS 2 Build Issues

1. Source ROS 2: `source /opt/ros/humble/setup.bash`
2. Install dependencies: `rosdep install --from-paths src --ignore-src -r -y`
3. Clean build: `rm -rf build install log && colcon build`

## Best Practices

1. **Use Configuration Files**: Always add packages to config files, not scripts
2. **Use Groups**: Create groups for common installation scenarios
3. **Test Changes**: Use `--dry-run` before installing
4. **Version Control**: Commit configuration changes
5. **Document**: Add comments in YAML files
6. **Virtual Environments**: Use venv for Python projects
7. **Pre-commit Hooks**: Run before committing code

