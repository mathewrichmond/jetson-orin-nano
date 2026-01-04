#!/bin/bash
# Build Package Script
# Creates an installable package from the repository

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
cd "$PROJECT_ROOT"

VERSION="${1:-$(git describe --tags --always --dirty 2>/dev/null || echo 'dev')}"
PACKAGE_NAME="isaac-robot"
PACKAGE_VERSION="${VERSION#v}"  # Remove 'v' prefix if present
PACKAGE_FULL_NAME="${PACKAGE_NAME}-${PACKAGE_VERSION}"
DIST_DIR="${PROJECT_ROOT}/dist"
PACKAGE_DIR="${DIST_DIR}/${PACKAGE_FULL_NAME}"

echo "=========================================="
echo "Building Package: ${PACKAGE_FULL_NAME}"
echo "=========================================="

# Clean and create directories
rm -rf "${DIST_DIR}"
mkdir -p "${PACKAGE_DIR}"
mkdir -p "${DIST_DIR}"

# Copy files to package directory
echo "Copying files..."

# Core files
cp -r config "${PACKAGE_DIR}/"
cp -r scripts "${PACKAGE_DIR}/"
cp -r src "${PACKAGE_DIR}/"
cp -r docs "${PACKAGE_DIR}/"
cp -r hardware "${PACKAGE_DIR}/" 2>/dev/null || true
cp -r monitoring "${PACKAGE_DIR}/" 2>/dev/null || true
cp -r logging "${PACKAGE_DIR}/" 2>/dev/null || true
cp -r control "${PACKAGE_DIR}/" 2>/dev/null || true

# Root files
cp setup.sh "${PACKAGE_DIR}/"
cp Makefile "${PACKAGE_DIR}/"
cp README.md "${PACKAGE_DIR}/"
cp CONTRIBUTING.md "${PACKAGE_DIR}/"
cp LICENSE "${PACKAGE_DIR}/"
cp requirements-dev.txt "${PACKAGE_DIR}/" 2>/dev/null || true
cp .editorconfig "${PACKAGE_DIR}/"
cp .cursorrules "${PACKAGE_DIR}/"
cp .pre-commit-config.yaml "${PACKAGE_DIR}/" 2>/dev/null || true
cp pyproject.toml "${PACKAGE_DIR}/" 2>/dev/null || true
cp .flake8 "${PACKAGE_DIR}/" 2>/dev/null || true
cp .pylintrc "${PACKAGE_DIR}/" 2>/dev/null || true

# VS Code config (optional)
if [ -d .vscode ]; then
    cp -r .vscode "${PACKAGE_DIR}/"
fi

# Create package metadata
cat > "${PACKAGE_DIR}/.package_info" << EOF
PACKAGE_NAME=${PACKAGE_NAME}
PACKAGE_VERSION=${PACKAGE_VERSION}
BUILD_DATE=$(date -u +"%Y-%m-%dT%H:%M:%SZ")
GIT_COMMIT=$(git rev-parse HEAD 2>/dev/null || echo "unknown")
GIT_BRANCH=$(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo "unknown")
EOF

# Create installation script
cat > "${PACKAGE_DIR}/install.sh" << 'INSTALL_EOF'
#!/bin/bash
# Package Installation Script

set -e

INSTALL_DIR="${ISAAC_INSTALL_DIR:-/opt/isaac-robot}"
DEV_DIR="${ISAAC_DEV_DIR:-/home/nano/src/jetson-orin-nano}"

echo "=========================================="
echo "Installing Isaac Robot Package"
echo "=========================================="

# Check if dev directory exists and warn
if [ -d "$DEV_DIR" ] && [ -f "$DEV_DIR/setup.sh" ]; then
    echo "WARNING: Dev sandbox found at $DEV_DIR"
    echo "Dev sandbox takes precedence over installed package."
    echo "Continue installation? (y/N)"
    read -r response
    if [[ ! "$response" =~ ^[Yy]$ ]]; then
        echo "Installation cancelled."
        exit 1
    fi
fi

# Create install directory
sudo mkdir -p "$INSTALL_DIR"
sudo chown $USER:$USER "$INSTALL_DIR"

# Copy package files
echo "Installing to $INSTALL_DIR..."
cp -r . "$INSTALL_DIR/"

# Set permissions
chmod +x "$INSTALL_DIR/setup.sh"
chmod +x "$INSTALL_DIR/scripts"/*/*.sh 2>/dev/null || true

# Create symlink for easy access
sudo ln -sf "$INSTALL_DIR/setup.sh" /usr/local/bin/isaac-setup

echo ""
echo "=========================================="
echo "Installation Complete!"
echo "=========================================="
echo ""
echo "Package installed to: $INSTALL_DIR"
echo "Run setup: isaac-setup"
echo ""
INSTALL_EOF

chmod +x "${PACKAGE_DIR}/install.sh"

# Create tarball
echo "Creating tarball..."
cd "${DIST_DIR}"
tar -czf "${PACKAGE_FULL_NAME}.tar.gz" "${PACKAGE_FULL_NAME}"

# Create checksum
sha256sum "${PACKAGE_FULL_NAME}.tar.gz" > "${PACKAGE_FULL_NAME}.tar.gz.sha256"

echo ""
echo "=========================================="
echo "Package Built Successfully!"
echo "=========================================="
echo "Package: ${PACKAGE_FULL_NAME}.tar.gz"
echo "Location: ${DIST_DIR}/"
echo "Size: $(du -h "${DIST_DIR}/${PACKAGE_FULL_NAME}.tar.gz" | cut -f1)"
echo ""
