#!/bin/bash
# Clean up old sessions and free disk space

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_DIR="$(cd "$SCRIPT_DIR/../../config/logging" && pwd)"

echo "═══════════════════════════════════════════════════════════════"
echo "  SESSION CLEANUP"
echo "═══════════════════════════════════════════════════════════════"
echo ""

# Load configuration
if [ -f "$CONFIG_DIR/session_structure.yaml" ]; then
    STORAGE_ROOT=$(python3 -c "
import yaml
with open('$CONFIG_DIR/session_structure.yaml') as f:
    config = yaml.safe_load(f)
    print(config['session_structure']['root'])
")
    KEEP_RAW_DAYS=$(python3 -c "
import yaml
with open('$CONFIG_DIR/session_structure.yaml') as f:
    config = yaml.safe_load(f)
    print(config['session_structure']['cleanup']['keep_raw_days'])
")
    KEEP_PROCESSED_DAYS=$(python3 -c "
import yaml
with open('$CONFIG_DIR/session_structure.yaml') as f:
    config = yaml.safe_load(f)
    print(config['session_structure']['cleanup']['keep_processed_days'])
")
    DELETE_FAILED_DAYS=$(python3 -c "
import yaml
with open('$CONFIG_DIR/session_structure.yaml') as f:
    config = yaml.safe_load(f)
    print(config['session_structure']['cleanup']['delete_failed_days'])
")
    ARCHIVE_THRESHOLD=$(python3 -c "
import yaml
with open('$CONFIG_DIR/session_structure.yaml') as f:
    config = yaml.safe_load(f)
    print(config['session_structure']['cleanup']['archive_threshold_percent'])
")
else
    STORAGE_ROOT="/home/nano/data"
    KEEP_RAW_DAYS=7
    KEEP_PROCESSED_DAYS=30
    DELETE_FAILED_DAYS=30
    ARCHIVE_THRESHOLD=85
fi

# Check if storage exists
if [ ! -d "$STORAGE_ROOT" ]; then
    echo "Storage directory $STORAGE_ROOT does not exist"
    exit 1
fi

# Check disk usage
DISK_USAGE=$(df "$STORAGE_ROOT" | tail -1 | awk '{print $5}' | tr -d '%')
echo "Current disk usage: $DISK_USAGE%"

if [ "$DISK_USAGE" -lt "$ARCHIVE_THRESHOLD" ]; then
    echo "✓ Disk usage below threshold ($ARCHIVE_THRESHOLD%), skipping cleanup"
    echo "═══════════════════════════════════════════════════════════════"
    exit 0
fi

echo "⚠️  Disk usage above threshold, starting cleanup..."
echo ""

# Archive old raw data (keep last N days)
echo "1. Archiving old raw data (older than $KEEP_RAW_DAYS days)..."
ARCHIVED_COUNT=0
find "$STORAGE_ROOT" -name "session_*" -type d -mtime +$KEEP_RAW_DAYS | while read session_dir; do
    RAW_DIR="$session_dir/raw"
    if [ -d "$RAW_DIR" ] && [ -n "$(ls -A $RAW_DIR 2>/dev/null)" ]; then
        SESSION_NAME=$(basename "$session_dir")
        echo "  Archiving raw data: $SESSION_NAME"
        
        # Check if processed data exists
        PROCESSED_DIR="$session_dir/processed"
        if [ -d "$PROCESSED_DIR" ] && [ -n "$(ls -A $PROCESSED_DIR 2>/dev/null)" ]; then
            # Safe to remove raw bags if processed data exists
            rm -rf "$RAW_DIR"/*
            echo "    ✓ Removed raw bags (processed data available)"
            ARCHIVED_COUNT=$((ARCHIVED_COUNT + 1))
        else
            echo "    ⚠️  No processed data, keeping raw bags"
        fi
    fi
done
echo "  Archived $ARCHIVED_COUNT sessions"
echo ""

# Delete old processed data (older than N days)
echo "2. Deleting old processed data (older than $KEEP_PROCESSED_DAYS days)..."
DELETED_PROCESSED=0
find "$STORAGE_ROOT" -name "session_*" -type d -mtime +$KEEP_PROCESSED_DAYS | while read session_dir; do
    PROCESSED_DIR="$session_dir/processed"
    if [ -d "$PROCESSED_DIR" ] && [ -n "$(ls -A $PROCESSED_DIR 2>/dev/null)" ]; then
        SESSION_NAME=$(basename "$session_dir")
        echo "  Deleting processed data: $SESSION_NAME"
        rm -rf "$PROCESSED_DIR"/*
        DELETED_PROCESSED=$((DELETED_PROCESSED + 1))
    fi
done
echo "  Deleted $DELETED_PROCESSED processed datasets"
echo ""

# Delete failed sessions (older than N days)
echo "3. Deleting failed sessions (older than $DELETE_FAILED_DAYS days)..."
DELETED_FAILED=0
find "$STORAGE_ROOT" -name "session_*" -type d -mtime +$DELETE_FAILED_DAYS | while read session_dir; do
    MANIFEST="$session_dir/metadata/session_manifest.json"
    if [ -f "$MANIFEST" ]; then
        STATUS=$(python3 -c "
import json
with open('$MANIFEST') as f:
    data = json.load(f)
    print(data.get('status', ''))
" 2>/dev/null || echo "")
        
        if [ "$STATUS" = "failed" ]; then
            SESSION_NAME=$(basename "$session_dir")
            echo "  Deleting failed session: $SESSION_NAME"
            rm -rf "$session_dir"
            DELETED_FAILED=$((DELETED_FAILED + 1))
        fi
    fi
done
echo "  Deleted $DELETED_FAILED failed sessions"
echo ""

# Delete empty session directories
echo "4. Cleaning up empty session directories..."
EMPTY_COUNT=0
find "$STORAGE_ROOT" -name "session_*" -type d -empty -delete 2>/dev/null || true
find "$STORAGE_ROOT" -name "session_*" -type d | while read session_dir; do
    # Check if only metadata directory exists
    DIR_COUNT=$(find "$session_dir" -mindepth 1 -maxdepth 1 -type d | wc -l)
    FILE_COUNT=$(find "$session_dir" -type f | wc -l)
    
    if [ "$FILE_COUNT" -le 1 ]; then  # Only manifest or less
        SESSION_NAME=$(basename "$session_dir")
        echo "  Removing empty session: $SESSION_NAME"
        rm -rf "$session_dir"
        EMPTY_COUNT=$((EMPTY_COUNT + 1))
    fi
done
echo "  Removed $EMPTY_COUNT empty sessions"
echo ""

# Check disk usage after cleanup
NEW_DISK_USAGE=$(df "$STORAGE_ROOT" | tail -1 | awk '{print $5}' | tr -d '%')
FREED=$((DISK_USAGE - NEW_DISK_USAGE))

echo "Cleanup complete!"
echo "  Disk usage: $DISK_USAGE% → $NEW_DISK_USAGE% (freed $FREED%)"
echo ""

if [ "$NEW_DISK_USAGE" -ge "$ARCHIVE_THRESHOLD" ]; then
    echo "⚠️  WARNING: Disk usage still above threshold"
    echo "   Consider manually moving data to cold storage"
fi

echo "═══════════════════════════════════════════════════════════════"
