# Migration to pyproject.toml

**Date**: 2026-01-27  
**Status**: ✅ Complete

---

## Overview

Migrated from `requirements.txt` / `requirements-dev.txt` to modern `pyproject.toml` for dependency management.

### Benefits

✅ **Single Source of Truth** - All project metadata in one file  
✅ **PEP 518/621 Standard** - Modern Python packaging standard  
✅ **Optional Dependencies** - Install only what you need  
✅ **Better Tooling** - Works with pip, poetry, pdm, hatch  
✅ **Editable Installs** - `pip install -e .` for development  

---

## Installation

### Basic Install (Runtime Only)

```bash
pip install -e .
```

Installs:
- pyyaml, numpy, psutil, python-dotenv

### Development Install (Recommended)

```bash
pip install -e ".[dev]"
```

Installs all dev tools:
- pytest, pytest-cov, pytest-mock, pytest-timeout
- black, isort, flake8, pylint, mypy
- pre-commit, sphinx, jupyter, etc.

### Specific Feature Sets

```bash
# Vision pipeline
pip install -e ".[vision]"

# Audio pipeline
pip install -e ".[audio]"

# VLA models (large, optional)
pip install -e ".[vla]"

# Hardware drivers
pip install -e ".[hardware]"

# Everything
pip install -e ".[all]"

# Dev + Vision + Audio
pip install -e ".[dev,vision,audio]"
```

---

## What Changed

### Before

```bash
# Multiple files
pip install -r requirements.txt
pip install -r requirements-dev.txt

# Manual tracking of dependencies
# No optional dependencies
# No project metadata
```

### After

```bash
# Single file (pyproject.toml)
pip install -e ".[dev]"

# Optional dependencies by feature
# Project metadata included
# Modern standard (PEP 518/621)
```

---

## pyproject.toml Structure

```toml
[build-system]
requires = ["setuptools>=61.0", "wheel"]
build-backend = "setuptools.build_meta"

[project]
name = "isaac-robot"
version = "0.1.0"
description = "..."
dependencies = [
    # Runtime dependencies
    "pyyaml>=6.0",
    "numpy>=1.24.0",
    # ...
]

[project.optional-dependencies]
dev = [...]        # Development tools
vision = [...]     # Vision dependencies
audio = [...]      # Audio dependencies
vla = [...]        # VLA model dependencies
hardware = [...]   # Hardware drivers
all = [...]        # Everything

[tool.black]       # Black configuration
[tool.isort]       # isort configuration
[tool.pytest.ini_options]  # pytest configuration
[tool.coverage.run]        # Coverage configuration
```

---

## CI/CD Updates

### GitHub Actions

**Before**:
```yaml
- name: Install dependencies
  run: |
    pip install pytest pytest-cov
    pip install -r requirements-dev.txt
```

**After**:
```yaml
- name: Install dependencies
  run: |
    pip install -e ".[dev]"
```

### Scripts

**Before**:
```bash
# In scripts/testing/run_tests.sh
pip install pytest pytest-cov pytest-mock pytest-timeout
```

**After**:
```bash
# In scripts/testing/run_tests.sh
pip install -e ".[dev]"
```

---

## Optional Dependencies

### dev
Development tools (pytest, black, isort, pylint, jupyter, etc.)

**Install**: `pip install -e ".[dev]"`

### vision
OpenCV, Pillow for vision pipeline

**Install**: `pip install -e ".[vision]"`

### audio
sounddevice, librosa, scipy for audio processing

**Install**: `pip install -e ".[audio]"`

### vla
PyTorch, transformers for VLA models (large downloads)

**Install**: `pip install -e ".[vla]"`

### hardware
ODrive, pyserial for hardware drivers

**Install**: `pip install -e ".[hardware]"`

### ros2
Reference list (installed via apt, not pip)

**Install**: Via apt (see setup.sh)

### all
All optional dependencies

**Install**: `pip install -e ".[all]"`

---

## Compatibility

### Backward Compatible

Old commands still work (if requirements-dev.txt kept):
```bash
pip install -r requirements-dev.txt
```

But **recommended** to migrate to:
```bash
pip install -e ".[dev]"
```

### setuptools vs poetry

This pyproject.toml uses **setuptools** as the backend:
```toml
[build-system]
requires = ["setuptools>=61.0", "wheel"]
build-backend = "setuptools.build_meta"
```

**Works with**:
- `pip install -e .` ✅
- `poetry install` ✅ (poetry reads pyproject.toml)
- `pdm install` ✅
- `hatch build` ✅

---

## Development Workflow

### Initial Setup

```bash
# Clone repository
git clone <repo>
cd jetson-orin-nano

# Install in development mode
pip install -e ".[dev]"

# Or with specific features
pip install -e ".[dev,vision,audio]"
```

### Adding Dependencies

**Runtime dependency**:
```toml
[project]
dependencies = [
    "pyyaml>=6.0",
    "new-package>=1.0.0",  # Add here
]
```

**Dev dependency**:
```toml
[project.optional-dependencies]
dev = [
    "pytest>=7.0.0",
    "new-dev-tool>=2.0.0",  # Add here
]
```

**Reinstall**:
```bash
pip install -e ".[dev]"
```

### Updating Dependencies

**Check for updates**:
```bash
pip list --outdated
```

**Update pyproject.toml**:
```toml
# Update version constraints
"pytest>=7.4.0",  # was >=7.0.0
```

**Reinstall**:
```bash
pip install --upgrade -e ".[dev]"
```

---

## Testing

All tests still work:

```bash
# Run tests
./scripts/testing/run_tests.sh all

# Tests now install dependencies from pyproject.toml
pip install -e ".[dev]"
pytest tests/
```

---

## Migration Checklist

✅ Created comprehensive pyproject.toml  
✅ Added [project] section with metadata  
✅ Migrated all dependencies from requirements-dev.txt  
✅ Added optional dependencies by feature  
✅ Updated GitHub Actions workflows  
✅ Updated pytest configuration in pyproject.toml  
✅ Kept tool configurations (black, isort, mypy, coverage)  
✅ Documented installation methods  
✅ Backward compatible (can keep requirements-dev.txt)  

---

## Removing Old Files (Optional)

After verifying everything works:

```bash
# Test with pyproject.toml
pip install -e ".[dev]"
./scripts/testing/run_tests.sh all

# If everything works, remove old files
rm requirements-dev.txt

# Commit
git rm requirements-dev.txt
git add pyproject.toml
git commit -m "refactor: Migrate to pyproject.toml for dependency management"
```

---

## Common Commands

```bash
# Install for development
pip install -e ".[dev]"

# Install with vision and audio
pip install -e ".[dev,vision,audio]"

# Install everything
pip install -e ".[all]"

# Uninstall
pip uninstall isaac-robot

# Reinstall (after changes)
pip install -e ".[dev]" --force-reinstall --no-deps

# Check installed version
pip show isaac-robot

# List dependencies
pip show isaac-robot | grep Requires
```

---

## Troubleshooting

### Issue: "No module named 'setuptools'"

**Fix**:
```bash
pip install --upgrade setuptools wheel
```

### Issue: "Could not build wheels"

**Fix**:
```bash
pip install --upgrade pip setuptools wheel
pip install -e ".[dev]"
```

### Issue: "Package not found"

**Fix**:
```bash
# Ensure you're in the repository root
cd /path/to/jetson-orin-nano

# Try reinstalling
pip uninstall isaac-robot
pip install -e ".[dev]"
```

### Issue: Old dependencies still installed

**Fix**:
```bash
# Create fresh environment
python -m venv .venv
source .venv/bin/activate
pip install -e ".[dev]"
```

---

## Best Practices

### 1. Use Editable Install

Always use `-e` flag for development:
```bash
pip install -e ".[dev]"
```

This makes code changes immediately available without reinstalling.

### 2. Version Constraints

Use `>=` for minimum versions:
```toml
"pytest>=7.0.0"  # Any version >= 7.0.0
```

Avoid pinning unless necessary:
```toml
"pytest==7.0.0"  # Pins to exact version (avoid)
```

### 3. Optional by Feature

Group related dependencies:
```toml
[project.optional-dependencies]
vision = ["opencv-python>=4.8.0"]
audio = ["librosa>=0.10.0"]
```

Install only what you need:
```bash
pip install -e ".[vision]"
```

### 4. Document Large Dependencies

For large optional dependencies (VLA models):
```toml
vla = [
    "torch>=2.0.0",  # Warning: Large download (~2GB)
    "transformers>=4.30.0",
]
```

---

## Reference

- **PEP 518**: https://peps.python.org/pep-0518/
- **PEP 621**: https://peps.python.org/pep-0621/
- **Packaging Guide**: https://packaging.python.org/en/latest/guides/writing-pyproject-toml/
- **setuptools**: https://setuptools.pypa.io/

---

**Migration Complete**: 2026-01-27  
**Status**: All systems using pyproject.toml
