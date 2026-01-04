#!/bin/bash
# Automated System Cleanup Script
# Cleans logs, temporary files, and manages disk space

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

LOG_FILE="${LOG_FILE:-/var/log/isaac-cleanup.log}"
DAYS_TO_KEEP="${DAYS_TO_KEEP:-30}"
DISK_THRESHOLD="${DISK_THRESHOLD:-85}"  # Percentage

log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $*" | tee -a "$LOG_FILE"
}

log "=========================================="
log "Isaac Robot System Cleanup"
log "=========================================="

# Check disk usage
DISK_USAGE=$(df -h / | awk 'NR==2 {print $5}' | sed 's/%//')
log "Disk usage: ${DISK_USAGE}%"

if [ "$DISK_USAGE" -ge "$DISK_THRESHOLD" ]; then
    log "WARNING: Disk usage above threshold (${DISK_THRESHOLD}%)"
fi

# Clean old logs
log "Cleaning old logs..."
find /var/log -name "*.log" -type f -mtime +${DAYS_TO_KEEP} -delete 2>/dev/null || true
find /var/log -name "*.gz" -type f -mtime +${DAYS_TO_KEEP} -delete 2>/dev/null || true
journalctl --vacuum-time=${DAYS_TO_KEEP}d > /dev/null 2>&1 || true

# Clean Isaac logs
if [ -d "$PROJECT_ROOT/logging" ]; then
    find "$PROJECT_ROOT/logging" -name "*.log" -type f -mtime +${DAYS_TO_KEEP} -delete 2>/dev/null || true
fi

# Clean ROS 2 logs
if [ -d ~/ros2_ws/log ]; then
    find ~/ros2_ws/log -type f -mtime +${DAYS_TO_KEEP} -delete 2>/dev/null || true
fi

# Clean temporary files
log "Cleaning temporary files..."
find /tmp -type f -mtime +7 -delete 2>/dev/null || true
find /var/tmp -type f -mtime +7 -delete 2>/dev/null || true

# Clean Python cache
log "Cleaning Python cache..."
find "$PROJECT_ROOT" -type d -name "__pycache__" -exec rm -r {} + 2>/dev/null || true
find "$PROJECT_ROOT" -type f -name "*.pyc" -delete 2>/dev/null || true

# Clean build artifacts (if disk space is low)
if [ "$DISK_USAGE" -ge 80 ]; then
    log "Disk space low, cleaning build artifacts..."
    rm -rf ~/ros2_ws/build/* 2>/dev/null || true
    rm -rf ~/ros2_ws/install/* 2>/dev/null || true
fi

# Clean package cache
if [ "$DISK_USAGE" -ge 85 ]; then
    log "Disk space critical, cleaning package cache..."
    if [ "$EUID" -eq 0 ]; then
        apt-get clean
        apt-get autoremove -y
    else
        sudo apt-get clean
        sudo apt-get autoremove -y
    fi
fi

# Report disk usage after cleanup
DISK_USAGE_AFTER=$(df -h / | awk 'NR==2 {print $5}' | sed 's/%//')
log "Disk usage after cleanup: ${DISK_USAGE_AFTER}%"

log "Cleanup complete"
log "=========================================="
