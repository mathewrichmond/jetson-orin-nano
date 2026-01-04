# Package Management Documentation

## Overview

The Isaac robot system uses a configuration-driven approach to package management. All packages are defined in YAML configuration files, and installation scripts use a common package manager utility to read from these configurations.

## Configuration Files

### System Packages

Location: `config/system/packages.yaml`

Defines all system packages (installed via `apt-get`) organized by category:

- `development` - Core development tools
- `python` - Python development packages
- `ros2` - ROS 2 development tools
- `cpp` - C++ development tools
- `debugging` - Debugging and profiling tools
- `documentation` - Documentation tools
- `containers` - Containerization tools
- `display` - Display and GUI packages
- `networking` - WiFi and networking tools
- `monitoring` - System monitoring tools

### Python Packages

Location: `config/system/python_packages.yaml`

Defines all Python packages (installed via `pip`) organized by category:

- `core` - Core development packages
- `testing` - Testing frameworks
- `code_quality` - Code quality tools
- `pre_commit` - Pre-commit hooks
- `task_runner` - Task runners
- `package_management` - Package managers
- `ros2_dev` - ROS 2 development
- `monitoring` - System monitoring
- `documentation` - Documentation tools
- `utilities` - Utility packages

## Package Groups

Both configuration files define **groups** that combine multiple categories:

### System Package Groups

- `dev_all` - All development packages
- `dev_minimal` - Minimal development setup
- `dev_full` - Full development environment
- `robot_essentials` - Essential packages for robot operation

### Python Package Groups

- `dev_all` - All development packages
- `dev_minimal` - Minimal development setup
- `code_quality_only` - Code quality tools only

## Package Manager Utility

Location: `scripts/utils/package_manager.py`

A Python utility that reads configuration files and installs packages.

### Usage

#### List Available Packages

```bash
# List system packages
python3 scripts/utils/package_manager.py list system

# List Python packages
python3 scripts/utils/package_manager.py list python
```

#### Install Packages

```bash
# Install system packages by group
python3 scripts/utils/package_manager.py install-system --groups dev_full

# Install system packages by category
python3 scripts/utils/package_manager.py install-system --categories development python

# Install Python packages by group
python3 scripts/utils/package_manager.py install-python --groups dev_all

# Install Python packages by category
python3 scripts/utils/package_manager.py install-python --categories testing code_quality

# Dry run (see what would be installed)
python3 scripts/utils/package_manager.py install-system --groups dev_minimal --dry-run
```

## Adding New Packages

### Adding System Packages

Edit `config/system/packages.yaml`:

```yaml
system:
  your_category:
    - package1
    - package2
    - package3
```

Then add to a group if desired:

```yaml
groups:
  your_group:
    - your_category
    - other_category
```

### Adding Python Packages

Edit `config/system/python_packages.yaml`:

```yaml
your_category:
  - package1>=1.0.0
  - package2>=2.0.0
```

Then add to a group if desired:

```yaml
groups:
  your_group:
    - your_category
    - other_category
```

## Benefits

1. **Single Source of Truth**: All packages defined in one place
2. **Easy Maintenance**: Add/remove packages without editing scripts
3. **Reusability**: Same configuration used by multiple scripts
4. **Flexibility**: Install specific groups or categories
5. **Dry Run**: Test installations without making changes
6. **Version Control**: Track package changes in git

## Example Workflows

### Install Minimal Development Environment

```bash
python3 scripts/utils/package_manager.py install-system --groups dev_minimal
python3 scripts/utils/package_manager.py install-python --groups dev_minimal
```

### Install Only Monitoring Tools

```bash
python3 scripts/utils/package_manager.py install-system --categories monitoring
python3 scripts/utils/package_manager.py install-python --categories monitoring
```

### Update Package Lists

1. Edit `config/system/packages.yaml` or `config/system/python_packages.yaml`
2. Test with dry run:
   ```bash
   python3 scripts/utils/package_manager.py install-system --groups dev_full --dry-run
   ```
3. Install:
   ```bash
   python3 scripts/utils/package_manager.py install-system --groups dev_full
   ```

## Integration with Setup Scripts

The unified setup script (`setup.sh`) uses the package manager:

- Automatically selects packages based on environment (Jetson/Docker/Ubuntu)
- Uses appropriate package groups for each environment
- Can be customized via command-line or configuration

## Best Practices

1. **Organize by Purpose**: Group related packages in categories
2. **Use Groups**: Create groups for common installation scenarios
3. **Version Pins**: Pin Python package versions when possible
4. **Document**: Add comments in YAML files for clarity
5. **Test**: Use dry-run before installing
6. **Commit Changes**: Track package changes in version control

