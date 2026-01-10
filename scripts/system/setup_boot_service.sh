#!/bin/bash
# Setup Robot Service for Boot (User Service)
# Installs and enables the isaac-robot.service as a user systemd service (no sudo required)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

echo "=========================================="
echo "Setting up Isaac Robot Service for Boot"
echo "=========================================="
echo ""

# Ensure systemd user services directory exists
SYSTEMD_USER_DIR="$HOME/.config/systemd/user"
mkdir -p "$SYSTEMD_USER_DIR"

# Set graph to robot
echo "Setting graph selection to 'robot'..."
"$PROJECT_ROOT/scripts/utils/select_graph.sh" robot

# Copy service file to user systemd directory
echo "Installing user service file..."
cp "$PROJECT_ROOT/config/systemd/isaac-robot.service" "$SYSTEMD_USER_DIR/"
chmod 644 "$SYSTEMD_USER_DIR/isaac-robot.service"

# Reload systemd user daemon
echo "Reloading systemd user daemon..."
systemctl --user daemon-reload

# Enable service for user session (persists across logins)
echo "Enabling service for user session..."
systemctl --user enable isaac-robot.service

# Enable lingering so service runs even when user is not logged in
echo "Enabling lingering (service runs without login)..."
loginctl enable-linger "$USER" 2>/dev/null || {
    echo "Note: Could not enable lingering (may require sudo)"
    echo "  Run: sudo loginctl enable-linger $USER"
}

echo ""
echo "=========================================="
echo "Setup Complete"
echo "=========================================="
echo ""
echo "Service Status:"
systemctl --user is-enabled isaac-robot.service && echo "  ✓ Service enabled for boot" || echo "  ✗ Service not enabled"
echo ""
echo "Graph Selection:"
cat "$PROJECT_ROOT/config/robot/selected_graph.txt" && echo ""
echo ""
echo "To start the service now:"
echo "  ./scripts/system/manage_graph.sh start robot"
echo "  # or: systemctl --user start isaac-robot.service"
echo ""
echo "To check status:"
echo "  ./scripts/system/manage_graph.sh status"
echo "  # or: systemctl --user status isaac-robot.service"
echo ""
echo "To view logs:"
echo "  ./scripts/system/manage_graph.sh logs"
echo "  # or: journalctl --user -u isaac-robot.service -f"
echo ""
