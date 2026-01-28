#!/bin/bash
# Install release package to production

set -e

RELEASE_TAG="${1:-latest}"
TARGET_USER="${2:-nano}"
TARGET_HOST="${3:-localhost}"
INSTALL_PATH="/opt/isaac-robot"

echo "═══════════════════════════════════════════════════════════════"
echo "  RELEASE INSTALLATION"
echo "═══════════════════════════════════════════════════════════════"
echo "  Release: $RELEASE_TAG"
echo "  Target: $TARGET_USER@$TARGET_HOST"
echo "  Install path: $INSTALL_PATH"
echo ""

# Download release package
echo "Downloading release package..."
RELEASE_URL="https://github.com/YOUR_ORG/jetson-orin-nano/releases/download/$RELEASE_TAG/isaac-robot-$RELEASE_TAG.tar.gz"
TEMP_FILE="/tmp/isaac-robot-$RELEASE_TAG.tar.gz"

wget -O "$TEMP_FILE" "$RELEASE_URL" || {
    echo "ERROR: Failed to download release"
    exit 1
}

echo "  ✓ Downloaded: $TEMP_FILE"

# Verify checksum
if [ -f "$TEMP_FILE.sha256" ]; then
    echo "Verifying checksum..."
    sha256sum -c "$TEMP_FILE.sha256" || {
        echo "ERROR: Checksum verification failed"
        exit 1
    }
    echo "  ✓ Checksum verified"
fi

# Install
echo ""
echo "Installing to $INSTALL_PATH..."

if [ "$TARGET_HOST" = "localhost" ]; then
    # Local installation
    sudo mkdir -p "$INSTALL_PATH"
    sudo tar -xzf "$TEMP_FILE" -C "$INSTALL_PATH" --strip-components=1
    sudo chown -R $TARGET_USER:$TARGET_USER "$INSTALL_PATH"
    echo "  ✓ Installed locally"
else
    # Remote installation
    scp "$TEMP_FILE" "$TARGET_USER@$TARGET_HOST:/tmp/"
    ssh "$TARGET_USER@$TARGET_HOST" "
        sudo mkdir -p $INSTALL_PATH &&
        sudo tar -xzf /tmp/$(basename $TEMP_FILE) -C $INSTALL_PATH --strip-components=1 &&
        sudo chown -R $TARGET_USER:$TARGET_USER $INSTALL_PATH
    "
    echo "  ✓ Installed remotely"
fi

# Update systemd service path (if needed)
echo ""
echo "Updating systemd service..."
SERVICE_FILE="$INSTALL_PATH/config/systemd/isaac-robot.service"
if [ "$TARGET_HOST" = "localhost" ]; then
    if [ -f "$SERVICE_FILE" ]; then
        sudo cp "$SERVICE_FILE" /etc/systemd/user/
        systemctl --user daemon-reload
        echo "  ✓ Service updated"
    fi
else
    ssh "$TARGET_USER@$TARGET_HOST" "
        if [ -f $SERVICE_FILE ]; then
            sudo cp $SERVICE_FILE /etc/systemd/user/
            systemctl --user daemon-reload
        fi
    "
    echo "  ✓ Service updated"
fi

# Restart service
echo ""
echo "Restarting production service..."
if [ "$TARGET_HOST" = "localhost" ]; then
    systemctl --user restart isaac-robot.service
else
    ssh "$TARGET_USER@$TARGET_HOST" "systemctl --user restart isaac-robot.service"
fi

# Wait and check health
sleep 5
echo ""
echo "Checking service health..."
if [ "$TARGET_HOST" = "localhost" ]; then
    systemctl --user is-active isaac-robot.service && echo "  ✓ Service running" || echo "  ❌ Service not running"
else
    ssh "$TARGET_USER@$TARGET_HOST" "systemctl --user is-active isaac-robot.service" && echo "  ✓ Service running" || echo "  ❌ Service not running"
fi

# Cleanup
rm -f "$TEMP_FILE"

echo ""
echo "✓ Release installation complete"
echo "  Release: $RELEASE_TAG"
echo "  Path: $INSTALL_PATH"
echo "═══════════════════════════════════════════════════════════════"
