# Repository Structure Guide

This document provides a detailed explanation of the repository structure and where to find or place different types of files.

## Top-Level Directories

### `scripts/` - Setup and Maintenance Scripts
All executable scripts for system setup, hardware installation, monitoring, and maintenance.

- **`scripts/system/`**: System-level setup scripts
  - `setup_isaac.sh` - Initial Jetson system setup (first boot only)
- **`scripts/hardware/`**: Hardware installation scripts
  - Realsense, motor controllers, etc.
- **`scripts/monitoring/`**: Health check and monitoring scripts
  - `system_health_check.sh` - System health verification
- **`scripts/maintenance/`**: Self-updating, cleaning, recovery scripts
  - Self-update, self-clean, self-restore scripts

### `src/` - Source Code
All source code for the robot system (ROS 2 packages, Python modules, etc.)

- **`src/vla_controller/`**: VLA model implementation
- **`src/hardware_drivers/`**: Hardware driver wrappers and ROS 2 interfaces
- **`src/control_modes/`**: Control mode implementations
- **`src/utils/`**: Shared utilities and helper functions

### `docs/` - Documentation
Comprehensive documentation for the system.

- **`docs/architecture/`**: System architecture and design documents
- **`docs/hardware/`**: Hardware setup and integration guides
- **`docs/setup/`**: Installation and setup documentation
- **`docs/api/`**: API documentation and references

### `config/` - Configuration Files
YAML, JSON, and other configuration files.

- **`config/system/`**: System-wide configurations
- **`config/hardware/`**: Hardware-specific configurations
- **`config/control/`**: Control mode configurations

### `hardware/` - Hardware Setup
Hardware-specific setup scripts, configs, and documentation.

- **`hardware/realsense/`**: Realsense camera setup
- **`hardware/motor_controllers/`**: Motor controller setup
- **`hardware/raspberry_pi_modules/`**: Raspberry Pi sub-module setup

### `monitoring/` - Monitoring Infrastructure
Monitoring nodes, scripts, and infrastructure.

- **`monitoring/system/`**: System resource monitoring
- **`monitoring/hardware/`**: Hardware health monitoring
- **`monitoring/performance/`**: Performance metrics

### `logging/` - Logging Infrastructure
Logging configuration and management.

- **`logging/config/`**: Logging configuration files
- **`logging/scripts/`**: Log rotation and management scripts

### `control/` - Control System
Control mode definitions and switching logic.

- **`control/modes/`**: Control mode definitions
- **`control/switching/`**: Mode switching logic and safety checks

## File Naming Conventions

- **Scripts**: `snake_case.sh` or `kebab-case.sh`
- **Python**: `snake_case.py`
- **ROS packages**: `snake_case` (lowercase, underscores)
- **Config files**: `kebab-case.yaml` or `snake_case.json`
- **Documentation**: `UPPERCASE.md` for main docs, `kebab-case.md` for topics

## Where to Put Things

### Adding New Hardware
1. Setup script → `scripts/hardware/install_<hardware>.sh`
2. Driver code → `src/hardware_drivers/<hardware>_driver/`
3. Config files → `config/hardware/<hardware>/`
4. Documentation → `docs/hardware/<hardware>.md`
5. Hardware-specific files → `hardware/<hardware>/`

### Adding New Control Mode
1. Implementation → `src/control_modes/<mode>/`
2. Configuration → `config/control/<mode>.yaml`
3. Mode definition → `control/modes/<mode>.yaml`
4. Documentation → `docs/architecture/control_modes.md`

### Adding System Scripts
1. System setup → `scripts/system/`
2. Hardware setup → `scripts/hardware/`
3. Monitoring → `scripts/monitoring/`
4. Maintenance → `scripts/maintenance/`

### Adding Documentation
1. Architecture → `docs/architecture/`
2. Hardware guides → `docs/hardware/`
3. Setup guides → `docs/setup/`
4. API docs → `docs/api/`

## ROS 2 Workspace Integration

Source code in `src/` is intended to be integrated into the ROS 2 workspace:
- ROS 2 workspace: `~/ros2_ws/`
- Link or copy packages: `~/ros2_ws/src/` → `~/src/jetson-orin-nano/src/`

## Best Practices

1. **Keep it organized**: Place files in appropriate directories
2. **Document everything**: Add README.md files in directories
3. **Follow conventions**: Use consistent naming and structure
4. **Version control**: Use `.gitignore` appropriately
5. **Safety first**: Consider safety implications for robot systems

## Questions?

See `README.md` for overview or `CONTRIBUTING.md` for development guidelines.
