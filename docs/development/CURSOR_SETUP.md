# Cursor IDE Setup Guide

## ZSH Terminal Configuration

ZSH has been added to Cursor's terminal profiles. To use it:

1. **Install ZSH** (if not already installed):
   ```bash
   sudo apt-get install zsh
   ```
   Or it will be installed automatically when you run `sudo ./setup.sh`

2. **Restart Cursor**:
   - Close Cursor completely
   - Reopen Cursor
   - Or use: View → Command Palette → "Reload Window"

3. **Select ZSH in Terminal**:
   - Open terminal: Press `` Ctrl+` `` or View → Terminal
   - Click the dropdown arrow next to the `+` button in the terminal panel
   - Select **"zsh"** from the list

### Make ZSH Default

To make zsh the default terminal in Cursor, edit `.vscode/settings.json`:
```json
"terminal.integrated.defaultProfile.linux": "zsh",
```

### Verify

After installing zsh and restarting Cursor:
- Open terminal dropdown
- You should see both "bash" and "zsh" options
- Select "zsh" to use it

### Troubleshooting

If zsh doesn't appear:
1. Make sure zsh is installed: `which zsh`
2. Restart Cursor completely
3. Check `.vscode/settings.json` has the zsh profile
4. Try reloading the window: Ctrl+Shift+P → "Reload Window"

---

# Cursor IDE Setup Guide

## Overview

This guide covers setting up Cursor IDE for development on the Isaac robot system. Cursor is compatible with VS Code configurations, so all VS Code settings apply.

## Quick Setup

1. **Open the project in Cursor**:
   ```bash
   cd ~/src/jetson-orin-nano
   cursor .
   ```

2. **Install recommended extensions**:
   - Cursor will prompt you to install recommended extensions
   - Or use: `Ctrl+Shift+X` and search for "Recommended"

3. **Select Python interpreter**:
   - Press `Ctrl+Shift+P`
   - Type "Python: Select Interpreter"
   - Choose `.venv/bin/python` (created by setup.sh)

## Configuration Files

The project includes comprehensive Cursor/VS Code configuration:

### `.vscode/settings.json`
- Python interpreter and linting
- ROS 2 workspace configuration
- Editor formatting and code actions
- File associations for ROS 2 files

### `.vscode/extensions.json`
- Recommended extensions for Python, ROS 2, Docker, etc.
- Automatically prompts to install when opening project

### `.vscode/tasks.json`
- Build tasks for ROS 2 workspace
- Python formatting and linting tasks
- Docker tasks
- System health checks

### `.vscode/launch.json`
- Python debugging configurations
- ROS 2 node debugging
- System monitor debugging

### `.vscode/c_cpp_properties.json`
- C/C++ IntelliSense configuration
- ROS 2 include paths
- Compiler settings

### `.cursorrules`
- AI assistant rules for Cursor
- Repository-specific context
- Coding guidelines

## Key Features

### Python Development

- **Auto-formatting**: Black formatter on save
- **Linting**: Flake8, Pylint, MyPy
- **Type checking**: Pylance with type hints
- **Import sorting**: isort integration

### ROS 2 Development

- **Workspace integration**: Automatic ROS 2 workspace detection
- **Launch file support**: Syntax highlighting for `.launch.py` files
- **Node debugging**: Debug ROS 2 nodes directly
- **Build tasks**: One-click ROS 2 workspace builds

### Docker Development

- **Docker Compose**: Integrated Docker Compose support
- **Container debugging**: Debug code running in containers
- **Remote containers**: Develop inside containers

## Tasks

Access tasks with `Ctrl+Shift+P` → "Tasks: Run Task":

- **Setup: Run unified setup** - Run `./setup.sh`
- **ROS 2: Build workspace** - Build ROS 2 workspace
- **ROS 2: Build package** - Build specific package
- **ROS 2: Test workspace** - Run tests
- **Python: Format code** - Format all Python files
- **Python: Lint code** - Run Flake8
- **Python: Type check** - Run MyPy
- **Pre-commit: Run all hooks** - Run pre-commit checks
- **Docker: Build image** - Build Docker image
- **System: Health check** - Run system health check

## Debugging

### Python Debugging

1. Set breakpoints in Python files
2. Press `F5` or use "Run and Debug"
3. Select "Python: Current File" or "Python: System Monitor Node"

### ROS 2 Debugging

1. Build workspace first (task)
2. Select "ROS 2: Launch System Monitor" configuration
3. Press `F5` to start debugging

## Remote Development

### SSH to Jetson

1. Install "Remote - SSH" extension
2. Connect to `isaac.local` or `nano@isaac.local`
3. Open project folder remotely

### Docker Development

1. Install "Dev Containers" extension
2. Open command palette: `Ctrl+Shift+P`
3. Select "Dev Containers: Reopen in Container"
4. Choose `docker-compose.yml`

## Recommended Workflow

1. **Open project**: `cursor .`
2. **Install extensions**: Accept recommended extensions
3. **Select Python interpreter**: `.venv/bin/python`
4. **Run setup**: Use task "Setup: Run unified setup"
5. **Build ROS 2**: Use task "ROS 2: Build workspace"
6. **Start debugging**: Press `F5` with appropriate configuration

## Tips

- **Quick Open**: `Ctrl+P` to quickly open files
- **Command Palette**: `Ctrl+Shift+P` for all commands
- **Terminal**: `Ctrl+`` (backtick) for integrated terminal
- **Multi-cursor**: `Ctrl+D` to select next occurrence
- **Format Document**: `Shift+Alt+F` (or auto-format on save)

## Troubleshooting

### Python Interpreter Not Found

1. Run `./setup.sh` to create virtual environment
2. Select interpreter: `Ctrl+Shift+P` → "Python: Select Interpreter"
3. Choose `.venv/bin/python`

### ROS 2 Not Detected

1. Ensure ROS 2 is installed: `source /opt/ros/humble/setup.bash`
2. Check workspace path in settings: `~/ros2_ws`
3. Install ROS extension: "ROS" by Microsoft

### Extensions Not Installing

1. Check internet connection
2. Try installing manually from marketplace
3. Check Cursor/VS Code version compatibility

## Additional Resources

- [Cursor Documentation](https://cursor.sh/docs)
- [VS Code Python Guide](https://code.visualstudio.com/docs/python/python-tutorial)
- [ROS 2 VS Code Extension](https://github.com/ms-iot/vscode-ros)
