# Packaging Scripts

This directory contains scripts for building and installing packages.

## Scripts

### `build_package.sh`

Builds an installable package from the repository.

**Usage**:
```bash
./scripts/packaging/build_package.sh [version]
```

**Output**:
- `dist/isaac-robot-<version>.tar.gz` - Package archive
- `dist/isaac-robot-<version>.tar.gz.sha256` - Checksum file

### `install_package.sh`

Installs a package tarball.

**Usage**:
```bash
sudo ./scripts/packaging/install_package.sh <package.tar.gz>
```

**Installation**:
- Extracts to `/opt/isaac-robot` (or `$ISAAC_INSTALL_DIR`)
- Creates symlink: `/usr/local/bin/isaac-setup`
- Warns if dev sandbox exists (dev takes precedence)

## Package Structure

Packages include:
- All source code (`src/`)
- Configuration files (`config/`)
- Scripts (`scripts/`)
- Documentation (`docs/`)
- Setup script (`setup.sh`)
- Installation script (`install.sh`)

## Package Priority

The system checks for Isaac root in this order:
1. Dev sandbox: `/home/nano/src/jetson-orin-nano`
2. Installed package: `/opt/isaac-robot`
3. Environment variables

See `scripts/utils/find_isaac_root.py` for implementation.
