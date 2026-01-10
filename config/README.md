# Configuration Files

This directory contains **all** configuration files for the Isaac robot system. All node configs must be stored here, organized by category.

## Structure

- `hardware/` - Hardware driver configurations
  - `realsense_params.yaml` - RealSense camera parameters
  - Other hardware configs...
- `system/` - System node configurations
  - `system_monitor_params.yaml` - System monitor parameters
  - Other system configs...
- `robot/` - Robot graph configurations
  - `robot_graph.yaml` - Default robot graph
  - `minimal_graph.yaml` - Minimal system graph
  - `full_graph.yaml` - Full system graph
- `control/` - Control mode configurations
- `visualization/` - Visualization configurations
  - `foxglove_cameras_layout.json` - Foxglove Studio camera layout
  - `foxglove_cameras_only_layout.json` - Camera-only layout
  - `foxglove_layout.json` - Original camera layout
- `systemd/` - Systemd service files

## Configuration Management

### Centralized Config Policy

**All node configurations must be stored in `config/`**, organized by category:
- Hardware configs → `config/hardware/`
- System configs → `config/system/`
- Robot graphs → `config/robot/`
- Control configs → `config/control/`
- Visualization configs → `config/visualization/`

### Config Resolution

Launch files resolve configs in this order:
1. Centralized `config/` directory (preferred)
2. Package share directory (fallback for backward compatibility)
3. Package source directory (legacy)

### Adding New Configs

1. Place config file in appropriate `config/<category>/` directory
2. Use descriptive name: `<node_name>_params.yaml` or `<component>_config.yaml`
3. Update launch files to use centralized config (see `config_finder.py` utility)
4. Document config parameters in node README

## File Formats

- **YAML**: Preferred for ROS 2 parameters and graph configs
- **JSON**: For structured data (if needed)
- **INI**: For simple key-value configs (avoid if possible)

## Usage

Configuration files are loaded by:
- ROS 2 launch files (via `config_finder.py` utility)
- Python/C++ applications
- System scripts

## Editing Configs

All configs in this directory are part of the global configuration and can be edited directly. Changes take effect on next node restart (or use `ros2 param set` for runtime changes).
