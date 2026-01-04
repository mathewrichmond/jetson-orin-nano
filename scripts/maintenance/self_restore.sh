#!/bin/bash
# Automated System Restore Script
# Attempts to restore system from common failure scenarios

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

LOG_FILE="${LOG_FILE:-/var/log/isaac-restore.log}"

log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $*" | tee -a "$LOG_FILE"
}

log "=========================================="
log "Isaac Robot System Restore"
log "=========================================="

# Find Isaac root
ISAAC_ROOT="${ISAAC_ROOT:-$(python3 "$PROJECT_ROOT/scripts/utils/find_isaac_root.py" 2>/dev/null || echo "$PROJECT_ROOT")}"

# Restore services if they're not running
log "Checking services..."
if ! systemctl is-active --quiet isaac-system-monitor.service; then
    log "Restarting isaac-system-monitor..."
    sudo systemctl restart isaac-system-monitor.service || log "Failed to restart"
fi

if ! systemctl is-active --quiet isaac-robot.service; then
    log "Restarting isaac-robot..."
    sudo systemctl restart isaac-robot.service || log "Failed to restart"
fi

# Restore ROS 2 workspace if needed
if [ ! -d ~/ros2_ws/src ]; then
    log "Restoring ROS 2 workspace..."
    mkdir -p ~/ros2_ws/src
    if [ -d "$ISAAC_ROOT/src/system_monitor" ]; then
        cd ~/ros2_ws/src
        ln -sf "$ISAAC_ROOT/src/system_monitor" system_monitor
    fi
fi

# Restore virtual environment if needed
if [ ! -d "$ISAAC_ROOT/.venv" ]; then
    log "Restoring virtual environment..."
    cd "$ISAAC_ROOT"
    python3 -m venv .venv
    source .venv/bin/activate
    pip install --upgrade pip setuptools wheel
    if [ -f requirements-dev.txt ]; then
        pip install -r requirements-dev.txt || true
    fi
    deactivate
fi

# Run health check
log "Running health check..."
"$ISAAC_ROOT/scripts/maintenance/health_check.sh" || true

log "Restore complete"
log "=========================================="
