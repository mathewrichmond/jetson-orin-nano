#!/bin/bash
# Start logging session (wrapper for systemd integration)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "═══════════════════════════════════════════════════════════════"
echo "  STARTING DATA LOGGING SESSION"
echo "═══════════════════════════════════════════════════════════════"
echo ""

# Check if there's already an active session
ACTIVE_SESSION=$(python3 "$SCRIPT_DIR/session_manager.py" active 2>/dev/null | grep "Active session:" | awk '{print $3}' || echo "")

if [ -n "$ACTIVE_SESSION" ]; then
    echo "⚠️  Active session already exists: $ACTIVE_SESSION"
    echo "   Using existing session"
    export SESSION_ID="$ACTIVE_SESSION"
else
    # Create new session
    echo "Creating new session..."
    SESSION_OUTPUT=$(python3 "$SCRIPT_DIR/session_manager.py" create --env robot)
    SESSION_ID=$(echo "$SESSION_OUTPUT" | grep "Session ID:" | awk '{print $3}')
    
    if [ -z "$SESSION_ID" ]; then
        echo "ERROR: Failed to create session"
        exit 1
    fi
    
    export SESSION_ID
    echo "✓ Created session: $SESSION_ID"
fi

# Save SESSION_ID to environment file for other processes
echo "export SESSION_ID=$SESSION_ID" > /tmp/current_session.env
echo "  Exported to: /tmp/current_session.env"
echo ""

# Start ROS bag recording
echo "Starting ROS bag recording..."
"$SCRIPT_DIR/record_session.sh"

echo ""
echo "✓ Logging session started successfully"
echo "  Session ID: $SESSION_ID"
echo "  Monitor with: $SCRIPT_DIR/session_status.sh"
echo "═══════════════════════════════════════════════════════════════"
