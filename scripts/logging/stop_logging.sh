#!/bin/bash
# Stop logging session and trigger data pipeline

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "═══════════════════════════════════════════════════════════════"
echo "  STOPPING DATA LOGGING SESSION"
echo "═══════════════════════════════════════════════════════════════"
echo ""

# Get active session
if [ -f "/tmp/current_session.env" ]; then
    source /tmp/current_session.env
fi

if [ -z "$SESSION_ID" ]; then
    ACTIVE_SESSION=$(python3 "$SCRIPT_DIR/session_manager.py" active 2>/dev/null | grep "Active session:" | awk '{print $3}' || echo "")
    if [ -n "$ACTIVE_SESSION" ]; then
        SESSION_ID="$ACTIVE_SESSION"
    else
        echo "⚠️  No active session found"
        exit 0
    fi
fi

echo "Stopping session: $SESSION_ID"

# Get session path
SESSION_PATH=$(python3 -c "
import sys
sys.path.insert(0, '$SCRIPT_DIR')
from session_manager import SessionManager
manager = SessionManager()
session_path = manager.find_session('$SESSION_ID')
if session_path:
    print(session_path)
else:
    sys.exit(1)
" 2>/dev/null)

if [ -z "$SESSION_PATH" ]; then
    echo "ERROR: Session $SESSION_ID not found"
    exit 1
fi

# Stop ROS bag recording
if [ -f "$SESSION_PATH/raw/.record_pid" ]; then
    RECORD_PID=$(cat "$SESSION_PATH/raw/.record_pid")
    echo "  Stopping ROS bag recording (PID: $RECORD_PID)..."
    
    if ps -p $RECORD_PID > /dev/null 2>&1; then
        kill -SIGINT $RECORD_PID 2>/dev/null || true
        
        # Wait for graceful shutdown
        sleep 2
        
        # Force kill if still running
        if ps -p $RECORD_PID > /dev/null 2>&1; then
            kill -9 $RECORD_PID 2>/dev/null || true
        fi
        
        echo "  ✓ Recording stopped"
    else
        echo "  ℹ️  Recording process already stopped"
    fi
    
    rm -f "$SESSION_PATH/raw/.record_pid"
fi

# Finalize session
echo "  Finalizing session..."
python3 "$SCRIPT_DIR/session_manager.py" finalize "$SESSION_ID" --status completed
echo "  ✓ Session finalized"

# Clean up environment file
rm -f /tmp/current_session.env

# Trigger data pipeline asynchronously
if [ -f "$SCRIPT_DIR/../data_pipeline/trigger_pipeline.sh" ]; then
    echo ""
    echo "Triggering data pipeline..."
    "$SCRIPT_DIR/../data_pipeline/trigger_pipeline.sh" "$SESSION_ID" &
    echo "  ✓ Pipeline queued (running in background)"
fi

echo ""
echo "✓ Logging session stopped"
echo "  Session ID: $SESSION_ID"
echo "  Location: $SESSION_PATH"
echo "  View status: $SCRIPT_DIR/session_status.sh"
echo "═══════════════════════════════════════════════════════════════"
