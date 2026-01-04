# Configuration-Driven Development

## Philosophy

The Isaac robot system uses a **configuration-driven** approach where:
- **Configuration files** define what to install/configure
- **Scripts** are generic and read from configurations
- **Single source of truth** for all dependencies and settings

## Benefits

1. **Maintainability**: Change packages in one place, not multiple scripts
2. **Consistency**: Same configuration used across all scripts
3. **Flexibility**: Easy to create custom installation profiles
4. **Testability**: Dry-run mode to test without making changes
5. **Version Control**: Track changes to dependencies in git

## Configuration Files

### Package Configurations

| File | Purpose | Format |
|------|---------|-------|
| `config/system/packages.yaml` | System packages (apt-get) | YAML |
| `config/system/python_packages.yaml` | Python packages (pip) | YAML |

### Code Quality Configurations

| File | Purpose |
|------|---------|
| `.pre-commit-config.yaml` | Pre-commit hooks |
| `.flake8` | Flake8 linting |
| `.pylintrc` | Pylint configuration |
| `pyproject.toml` | Black, isort, pytest |
| `.editorconfig` | Editor settings |

## Package Manager

The `scripts/utils/package_manager.py` utility reads configurations and installs packages.

### Features

- **Groups**: Install collections of categories
- **Categories**: Install specific package categories
- **Dry Run**: Test without installing
- **List**: See available packages/groups

### Example Usage

```bash
# List what's available
python3 scripts/utils/package_manager.py list system
python3 scripts/utils/package_manager.py list python

# Install by group
python3 scripts/utils/package_manager.py install-system --groups dev_full

# Install by category
python3 scripts/utils/package_manager.py install-system --categories monitoring

# Dry run
python3 scripts/utils/package_manager.py install-system --groups dev_minimal --dry-run
```

## Adding Packages

### Step 1: Edit Configuration

Add to `config/system/packages.yaml`:

```yaml
system:
  your_category:
    - package1
    - package2
```

### Step 2: (Optional) Add to Group

```yaml
groups:
  your_group:
    - your_category
```

### Step 3: Install

```bash
python3 scripts/utils/package_manager.py install-system --categories your_category
```

## Scripts Using Configuration

The unified setup script (`setup.sh`) uses the package manager:

- Main entry point: `./setup.sh`
- Automatically detects environment and installs appropriate packages
- All package installation goes through the package manager
- Future scripts can use the same utility

## Best Practices

1. **Never hardcode packages in scripts** - Always use configuration files
2. **Use groups for common scenarios** - Makes installation easier
3. **Test with dry-run** - Verify before installing
4. **Document categories** - Add comments in YAML files
5. **Version control** - Commit configuration changes
6. **Keep scripts generic** - Scripts should read from config, not define packages

## Migration Guide

If you have existing scripts with hardcoded packages:

1. Extract package lists to `config/system/packages.yaml`
2. Organize into categories
3. Create groups if needed
4. Update script to use `package_manager.py`
5. Test with `--dry-run`
6. Remove hardcoded lists from script

## Example: Adding a New Tool

### Before (Hardcoded)

```bash
# In some script
sudo apt-get install -y new-tool1 new-tool2
```

### After (Configuration-Driven)

1. Edit `config/system/packages.yaml`:
```yaml
system:
  new_tools:
    - new-tool1
    - new-tool2
```

2. Script uses package manager:
```bash
python3 scripts/utils/package_manager.py install-system --categories new_tools
```

## Future Enhancements

- [ ] Support for other package managers (snap, flatpak)
- [ ] Configuration validation
- [ ] Dependency resolution
- [ ] Package version pinning for system packages
- [ ] Rollback capability
- [ ] Configuration templates for different scenarios

