# Utility Scripts

This directory contains reusable utility scripts used throughout the project.

## Package Manager

`package_manager.py` - Configuration-driven package installation utility

Reads package definitions from YAML configuration files and installs them.

### Usage

```bash
# List available packages
python3 scripts/utils/package_manager.py list system
python3 scripts/utils/package_manager.py list python

# Install packages by group
python3 scripts/utils/package_manager.py install-system --groups dev_full
python3 scripts/utils/package_manager.py install-python --groups dev_all

# Install packages by category
python3 scripts/utils/package_manager.py install-system --categories development python
python3 scripts/utils/package_manager.py install-python --categories testing

# Dry run (see what would be installed)
python3 scripts/utils/package_manager.py install-system --groups dev_minimal --dry-run
```

See `docs/development/PACKAGE_MANAGEMENT.md` for full documentation.

## Adding New Utilities

When adding new utility scripts:

1. Place them in this directory
2. Make them executable: `chmod +x script_name`
3. Add a shebang: `#!/usr/bin/env python3` or `#!/bin/bash`
4. Document usage in this README
5. Add to `.gitignore` if they generate temporary files

