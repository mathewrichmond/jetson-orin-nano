#!/bin/bash
# Install Package Script
# Installs a package tarball

set -e

if [ $# -lt 1 ]; then
    echo "Usage: $0 <package.tar.gz>"
    exit 1
fi

PACKAGE_FILE="$1"
INSTALL_DIR="${ISAAC_INSTALL_DIR:-/opt/isaac-robot}"
DEV_DIR="${ISAAC_DEV_DIR:-/home/nano/src/jetson-orin-nano}"

echo "=========================================="
echo "Installing Isaac Robot Package"
echo "=========================================="

# Check if package exists
if [ ! -f "$PACKAGE_FILE" ]; then
    echo "Error: Package file not found: $PACKAGE_FILE"
    exit 1
fi

# Check if dev directory exists
if [ -d "$DEV_DIR" ] && [ -f "$DEV_DIR/setup.sh" ]; then
    echo "WARNING: Dev sandbox found at $DEV_DIR"
    echo "Dev sandbox will take precedence over installed package."
    echo ""
fi

# Extract package
TEMP_DIR=$(mktemp -d)
echo "Extracting package..."
tar -xzf "$PACKAGE_FILE" -C "$TEMP_DIR"

# Find package directory
PACKAGE_DIR=$(find "$TEMP_DIR" -mindepth 1 -maxdepth 1 -type d | head -1)

# Create install directory
sudo mkdir -p "$INSTALL_DIR"
sudo chown $USER:$USER "$INSTALL_DIR"

# Copy files
echo "Installing to $INSTALL_DIR..."
cp -r "$PACKAGE_DIR"/* "$INSTALL_DIR/"

# Set permissions
chmod +x "$INSTALL_DIR/setup.sh"
find "$INSTALL_DIR/scripts" -name "*.sh" -exec chmod +x {} \;

# Create symlink
sudo ln -sf "$INSTALL_DIR/setup.sh" /usr/local/bin/isaac-setup

# Cleanup
rm -rf "$TEMP_DIR"

echo ""
echo "=========================================="
echo "Installation Complete!"
echo "=========================================="
echo ""
echo "Package installed to: $INSTALL_DIR"
echo "Run setup: isaac-setup"
echo ""
