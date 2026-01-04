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

# Copy service files
echo "Installing service files..."
cp "${PROJECT_ROOT}/config/systemd/isaac-robot.service" /etc/systemd/system/
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

# Enable services
echo "Enabling services..."
systemctl enable isaac-system-monitor.service
systemctl enable isaac-robot.service

# Start services
echo "Starting services..."
systemctl start isaac-system-monitor.service || echo "Note: isaac-system-monitor may need ROS 2 setup"
systemctl start isaac-robot.service || echo "Note: isaac-robot may need ROS 2 setup"

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
