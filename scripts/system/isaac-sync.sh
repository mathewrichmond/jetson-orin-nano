#!/bin/bash
# Isaac Robot Data Sync Script
# Syncs logs and config to remote when network is available
#
# Install to: /usr/local/bin/isaac-sync.sh
# Run by: systemd timer (isaac-sync.timer)

set -e

LOG_FILE="/var/log/isaac-sync.log"
DATA_DIR="/data"

log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $*" | tee -a "$LOG_FILE"
}

# Check if network is available
if ! ping -c 1 -W 2 8.8.8.8 > /dev/null 2>&1; then
    log "No network connectivity - skipping sync"
    exit 0
fi

log "Starting data sync..."

# Configure your remote server here
REMOTE_USER="your-user"
REMOTE_HOST="your-backup-server.local"
REMOTE_PATH="/backup/isaac"

# Uncomment and configure when ready
# rsync -avz --delete "$DATA_DIR/logs/" "$REMOTE_USER@$REMOTE_HOST:$REMOTE_PATH/logs/" >> "$LOG_FILE" 2>&1
# rsync -avz --delete "$DATA_DIR/config/" "$REMOTE_USER@$REMOTE_HOST:$REMOTE_PATH/config/" >> "$LOG_FILE" 2>&1
# rsync -avz --delete "$DATA_DIR/worldgraph/" "$REMOTE_USER@$REMOTE_HOST:$REMOTE_PATH/worldgraph/" >> "$LOG_FILE" 2>&1

log "Sync complete (configure remote server in this script)"

# For now, just log that sync would run
log "Sync configured for: $DATA_DIR/{logs,config,worldgraph}"
log "To enable: Edit /usr/local/bin/isaac-sync.sh with your remote server details"

# Note: config/ includes calibration data
# Calibration is backed up to remote for disaster recovery
# calibration_history/ preserves old calibrations
