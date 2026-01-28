#!/bin/bash
# Show current session status and disk usage

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_DIR="$(cd "$SCRIPT_DIR/../../config/logging" && pwd)"

# Load configuration
if [ -f "$CONFIG_DIR/session_structure.yaml" ]; then
    STORAGE_ROOT=$(grep "root:" "$CONFIG_DIR/session_structure.yaml" | awk '{print $2}' | tr -d '"')
    LOCAL_FALLBACK=$(grep "local_fallback:" "$CONFIG_DIR/session_structure.yaml" | awk '{print $2}' | tr -d '"')
else
    STORAGE_ROOT="/mnt/nfs/robot_data"
    LOCAL_FALLBACK="/home/nano/data"
fi

# Determine which storage is in use
if mountpoint -q "$STORAGE_ROOT" 2>/dev/null; then
    ACTIVE_STORAGE="$STORAGE_ROOT"
    STORAGE_TYPE="NFS"
else
    ACTIVE_STORAGE="$LOCAL_FALLBACK"
    STORAGE_TYPE="Local"
fi

echo "═══════════════════════════════════════════════════════════════"
echo "  SESSION STATUS REPORT"
echo "═══════════════════════════════════════════════════════════════"
echo ""

# Active session
echo "Active Session:"
echo "───────────────────────────────────────────────────────────────"
ACTIVE_SESSION=$(python3 "$SCRIPT_DIR/session_manager.py" active 2>/dev/null | grep "Active session:" | awk '{print $3}')
if [ -n "$ACTIVE_SESSION" ]; then
    python3 "$SCRIPT_DIR/session_manager.py" active
else
    echo "  No active recording session"
fi
echo ""

# Storage information
echo "Storage:"
echo "───────────────────────────────────────────────────────────────"
echo "  Type: $STORAGE_TYPE"
echo "  Location: $ACTIVE_STORAGE"

if [ -d "$ACTIVE_STORAGE" ]; then
    DISK_INFO=$(df -h "$ACTIVE_STORAGE" | tail -1)
    DISK_USAGE=$(echo "$DISK_INFO" | awk '{print $5}' | tr -d '%')
    DISK_AVAILABLE=$(echo "$DISK_INFO" | awk '{print $4}')
    DISK_USED=$(echo "$DISK_INFO" | awk '{print $3}')
    DISK_TOTAL=$(echo "$DISK_INFO" | awk '{print $2}')
    
    echo "  Used: $DISK_USED / $DISK_TOTAL ($DISK_USAGE%)"
    echo "  Available: $DISK_AVAILABLE"
    
    # Warn if disk usage is high
    if [ "$DISK_USAGE" -gt 85 ]; then
        echo "  ⚠️  WARNING: Disk usage above 85%!"
    fi
else
    echo "  ⚠️  Storage directory not accessible"
fi
echo ""

# Recent sessions
echo "Recent Sessions (last 5):"
echo "───────────────────────────────────────────────────────────────"
python3 "$SCRIPT_DIR/list_sessions.py" --limit 5 2>/dev/null || echo "  Unable to list sessions"
echo ""

# Pipeline processing queue
echo "Pipeline Processing:"
echo "───────────────────────────────────────────────────────────────"
PROCESSING_COUNT=$(find "$ACTIVE_STORAGE" -name "session_*" -type d -exec test -f {}/.processing_lock \; -print 2>/dev/null | wc -l)
if [ "$PROCESSING_COUNT" -gt 0 ]; then
    echo "  Sessions in processing: $PROCESSING_COUNT"
    find "$ACTIVE_STORAGE" -name "session_*" -type d -exec test -f {}/.processing_lock \; -print 2>/dev/null | while read session_dir; do
        session_name=$(basename "$session_dir")
        echo "    - $session_name"
    done
else
    echo "  No sessions currently processing"
fi
echo ""

# Disk usage estimate
if [ -d "$ACTIVE_STORAGE" ] && [ -n "$ACTIVE_SESSION" ]; then
    echo "Disk Space Estimate:"
    echo "───────────────────────────────────────────────────────────────"
    
    # Calculate average recording rate (bytes per second)
    SESSION_SIZE=$(du -sb "$ACTIVE_STORAGE/session_*_$ACTIVE_SESSION" 2>/dev/null | awk '{print $1}' || echo "0")
    START_TIME=$(python3 -c "import json; f=open('$ACTIVE_STORAGE/session_*_$ACTIVE_SESSION/metadata/session_manifest.json'); d=json.load(f); print(d['start_time'])" 2>/dev/null || echo "")
    
    if [ -n "$START_TIME" ] && [ "$SESSION_SIZE" -gt 0 ]; then
        CURRENT_TIME=$(date -u +%s)
        START_EPOCH=$(date -d "$START_TIME" +%s 2>/dev/null || echo "$CURRENT_TIME")
        ELAPSED=$((CURRENT_TIME - START_EPOCH))
        
        if [ "$ELAPSED" -gt 0 ]; then
            RATE_BYTES_PER_SEC=$((SESSION_SIZE / ELAPSED))
            RATE_MB_PER_MIN=$(echo "scale=1; $RATE_BYTES_PER_SEC * 60 / 1048576" | bc)
            
            # Calculate time until disk full
            AVAILABLE_BYTES=$(df --output=avail "$ACTIVE_STORAGE" | tail -1)
            AVAILABLE_BYTES=$((AVAILABLE_BYTES * 1024))  # Convert from KB to bytes
            
            if [ "$RATE_BYTES_PER_SEC" -gt 0 ]; then
                TIME_REMAINING_SEC=$((AVAILABLE_BYTES / RATE_BYTES_PER_SEC))
                TIME_REMAINING_MIN=$((TIME_REMAINING_SEC / 60))
                TIME_REMAINING_HOURS=$((TIME_REMAINING_MIN / 60))
                
                echo "  Recording rate: ~${RATE_MB_PER_MIN} MB/min"
                echo "  Time until disk full: ~${TIME_REMAINING_HOURS}h (${TIME_REMAINING_MIN}m)"
            fi
        fi
    fi
fi

echo "═══════════════════════════════════════════════════════════════"
