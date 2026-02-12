#!/bin/bash
# Install Systemd Services and Timers
# Installs systemd service files and timers for auto-start and maintenance

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

if [ "$EUID" -ne 0 ]; then
    echo "Please run as root or with sudo"
    exit 1
fi

echo "=========================================="
echo "Installing Systemd Services and Timers"
echo "=========================================="

# Configure hardware permissions (serial port access)
echo "Configuring hardware permissions..."
ISAAC_USER="${SUDO_USER:-$USER}"
if [ -z "$ISAAC_USER" ] || [ "$ISAAC_USER" = "root" ]; then
    # Try to detect the actual user
    ISAAC_USER=$(logname 2>/dev/null || echo "nano")
fi

echo "Adding user '$ISAAC_USER' to hardware access groups..."

# Add to dialout group for serial port access
if id -nG "$ISAAC_USER" | grep -qw "dialout"; then
    echo "  ✓ User '$ISAAC_USER' is already in dialout group"
else
    usermod -a -G dialout "$ISAAC_USER"
    echo "  ✓ User '$ISAAC_USER' added to dialout group"
fi

# Add to i2c group for I2C access (PHAT motor controller accelerometer)
if id -nG "$ISAAC_USER" | grep -qw "i2c"; then
    echo "  ✓ User '$ISAAC_USER' is already in i2c group"
else
    usermod -a -G i2c "$ISAAC_USER"
    echo "  ✓ User '$ISAAC_USER' added to i2c group"
fi

# Add to gpio group for GPIO access (PHAT motor controller)
if id -nG "$ISAAC_USER" | grep -qw "gpio"; then
    echo "  ✓ User '$ISAAC_USER' is already in gpio group"
else
    usermod -a -G gpio "$ISAAC_USER"
    echo "  ✓ User '$ISAAC_USER' added to gpio group"
fi

echo "  Note: User must logout/login or run 'newgrp <group>' for changes to take effect"

# Install robot service as user service (must run as the user, not root)
echo "Installing robot service as user service..."
# Enable lingering first so user services can run without login
loginctl enable-linger "$ISAAC_USER" 2>/dev/null || echo "  Note: Could not enable lingering (may need manual setup)"

# Get user's home directory
ISAAC_HOME=$(getent passwd "$ISAAC_USER" | cut -d: -f6)

# Run as the actual user to access their systemd user session
# We need to ensure the user's systemd user manager is available
if [ "$ISAAC_USER" != "root" ] && [ -n "$ISAAC_USER" ] && [ -n "$ISAAC_HOME" ]; then
    # Use runuser with proper environment setup for systemd user session
    # Set XDG_RUNTIME_DIR to the user's runtime directory
    XDG_RUNTIME_DIR="/run/user/$(id -u "$ISAAC_USER")"

    if command -v runuser >/dev/null 2>&1; then
        # Try runuser with proper environment
        runuser -l "$ISAAC_USER" -c "export HOME='$ISAAC_HOME'; export XDG_RUNTIME_DIR='$XDG_RUNTIME_DIR'; cd '${PROJECT_ROOT}' && '${PROJECT_ROOT}/scripts/system/setup_boot_service.sh'" || {
            echo "Warning: Could not install user service as $ISAAC_USER using runuser"
            echo "  This is normal if the user's systemd session isn't initialized"
            echo "  The service file will be installed, but you may need to run:"
            echo "    ./scripts/system/setup_boot_service.sh"
            echo "  Or manually enable the service after logging in:"
            echo "    systemctl --user daemon-reload"
            echo "    systemctl --user enable isaac-robot.service"
        }
    else
        # Fallback to su if runuser not available
        su - "$ISAAC_USER" -c "export XDG_RUNTIME_DIR='$XDG_RUNTIME_DIR'; cd '${PROJECT_ROOT}' && '${PROJECT_ROOT}/scripts/system/setup_boot_service.sh'" || {
            echo "Warning: Could not install user service as $ISAAC_USER"
            echo "  Please run manually: ./scripts/system/setup_boot_service.sh"
        }
    fi
else
    echo "Warning: Could not determine user for user service installation"
    echo "  Please run manually: ./scripts/system/setup_boot_service.sh"
fi

# Copy other service files (system services for maintenance)
echo "Installing maintenance service files..."
cp "${PROJECT_ROOT}/config/systemd/isaac-system-monitor.service" /etc/systemd/system/
cp "${PROJECT_ROOT}/config/systemd/isaac-update.service" /etc/systemd/system/
cp "${PROJECT_ROOT}/config/systemd/isaac-cleanup.service" /etc/systemd/system/
cp "${PROJECT_ROOT}/config/systemd/isaac-health-check.service" /etc/systemd/system/

# Copy timer files
echo "Installing timer files..."
cp "${PROJECT_ROOT}/config/systemd/isaac-update.timer" /etc/systemd/system/
cp "${PROJECT_ROOT}/config/systemd/isaac-cleanup.timer" /etc/systemd/system/
cp "${PROJECT_ROOT}/config/systemd/isaac-health-check.timer" /etc/systemd/system/

# Update service file paths if needed
sed -i "s|/home/nano/src/jetson-orin-nano|${PROJECT_ROOT}|g" /etc/systemd/system/isaac-*.service

# Create log directory
mkdir -p /var/log
touch /var/log/isaac-update.log
touch /var/log/isaac-cleanup.log
touch /var/log/isaac-health.log
chown nano:nano /var/log/isaac-*.log 2>/dev/null || true

# Reload systemd
systemctl daemon-reload

# Enable and start timers
echo "Enabling maintenance timers..."
systemctl enable isaac-update.timer
systemctl enable isaac-cleanup.timer
systemctl enable isaac-health-check.timer

# Start timers
systemctl start isaac-update.timer
systemctl start isaac-cleanup.timer
systemctl start isaac-health-check.timer

# Enable maintenance services
echo "Enabling maintenance services..."
systemctl enable isaac-system-monitor.service

# Start maintenance services
echo "Starting maintenance services..."
systemctl start isaac-system-monitor.service || echo "Note: isaac-system-monitor may need ROS 2 setup"

# Note: Robot service is managed via user service (already set up above)
echo ""
echo "Robot service installed as user service (no sudo required)"
echo "  Use: ./scripts/system/manage_graph.sh start robot"

echo ""
echo "=========================================="
echo "Services and Timers Installed!"
echo "=========================================="
echo ""
echo "Services enabled:"
echo "  - isaac-system-monitor.service"
echo "  - isaac-robot.service"
echo ""
echo "Maintenance timers enabled:"
echo "  - isaac-update.timer (daily at 3 AM)"
echo "  - isaac-cleanup.timer (weekly on Sunday at 2 AM)"
echo "  - isaac-health-check.timer (every 15 minutes)"
echo ""
echo "To check timer status:"
echo "  systemctl list-timers isaac-*"
echo ""
echo "To check service status:"
echo "  systemctl status isaac-robot"
echo "  systemctl status isaac-system-monitor"
echo ""
echo "To view logs:"
echo "  journalctl -u isaac-robot -f"
echo "  tail -f /var/log/isaac-health.log"
echo ""
