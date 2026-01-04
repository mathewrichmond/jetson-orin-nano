# VS Code / Cursor Configuration

This directory contains workspace settings for VS Code and Cursor IDE.

## Files

- `settings.json` - Workspace settings (Python, ROS 2, editor, etc.)
- `extensions.json` - Recommended extensions
- `tasks.json` - Build and development tasks
- `launch.json` - Debugging configurations
- `c_cpp_properties.json` - C/C++ IntelliSense settings
- `remote.json` - Remote development settings
- `ros.code-workspace` - Multi-root workspace for ROS 2

## Usage

1. Open project in Cursor/VS Code: `cursor .` or `code .`
2. Install recommended extensions when prompted
3. Select Python interpreter: `.venv/bin/python`
4. Use tasks (`Ctrl+Shift+P` → "Tasks: Run Task") for common operations

See `docs/development/CURSOR_SETUP.md` for detailed setup guide.
