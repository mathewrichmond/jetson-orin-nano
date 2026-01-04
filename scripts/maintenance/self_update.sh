#!/bin/bash
# Automated System Update Script
# Updates system packages, Python packages, and optionally the Isaac robot package

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Find Isaac root
ISAAC_ROOT="${ISAAC_ROOT:-$(python3 "$PROJECT_ROOT/scripts/utils/find_isaac_root.py" 2>/dev/null || echo "$PROJECT_ROOT")}"

LOG_FILE="${LOG_FILE:-/var/log/isaac-update.log}"
UPDATE_TYPE="${UPDATE_TYPE:-all}"  # all, system, python, isaac

log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $*" | tee -a "$LOG_FILE"
}

log "=========================================="
log "Isaac Robot System Update"
log "Update type: $UPDATE_TYPE"
log "=========================================="

# Update system packages
if [ "$UPDATE_TYPE" = "all" ] || [ "$UPDATE_TYPE" = "system" ]; then
    log "Updating system packages..."
    if [ "$EUID" -eq 0 ]; then
        apt-get update
        apt-get upgrade -y
        apt-get autoremove -y
        apt-get autoclean
        log "System packages updated"
    else
        sudo apt-get update
        sudo apt-get upgrade -y
        sudo apt-get autoremove -y
        sudo apt-get autoclean
        log "System packages updated"
    fi
fi

# Update Python packages
if [ "$UPDATE_TYPE" = "all" ] || [ "$UPDATE_TYPE" = "python" ]; then
    log "Updating Python packages..."
    pip3 install --user --upgrade pip setuptools wheel || true

    # Update packages from config if package manager exists
    if [ -f "$ISAAC_ROOT/scripts/utils/package_manager.py" ]; then
        python3 "$ISAAC_ROOT/scripts/utils/package_manager.py" install-python --groups dev_all || true
    fi

    log "Python packages updated"
fi

# Update Isaac robot package (if installed from package)
if [ "$UPDATE_TYPE" = "all" ] || [ "$UPDATE_TYPE" = "isaac" ]; then
    if [ "$ISAAC_ROOT" != "/home/nano/src/jetson-orin-nano" ]; then
        log "Checking for Isaac package updates..."

        # Check GitHub releases for new version
        CURRENT_VERSION=$(cat "$ISAAC_ROOT/.package_info" 2>/dev/null | grep PACKAGE_VERSION | cut -d= -f2 || echo "unknown")
        LATEST_VERSION=$(curl -s https://api.github.com/repos/$GITHUB_REPO/releases/latest 2>/dev/null | grep tag_name | cut -d'"' -f4 | sed 's/v//' || echo "unknown")

        if [ "$LATEST_VERSION" != "unknown" ] && [ "$LATEST_VERSION" != "$CURRENT_VERSION" ]; then
            log "New version available: $LATEST_VERSION (current: $CURRENT_VERSION)"
            log "Download and install manually or configure auto-update"
        else
            log "Isaac package is up to date"
        fi
    else
        log "Running from dev sandbox, skipping package update"
    fi
fi

log "Update complete"
log "=========================================="
