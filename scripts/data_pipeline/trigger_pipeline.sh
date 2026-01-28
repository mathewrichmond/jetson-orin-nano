#!/bin/bash
# Trigger data processing pipeline for a session

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Check arguments
if [ -z "$1" ]; then
    echo "Usage: $0 <session_id>"
    echo ""
    echo "Triggers the data processing pipeline for a session."
    echo "Runs at low priority to avoid impacting robot operation."
    exit 1
fi

SESSION_ID="$1"

echo "═══════════════════════════════════════════════════════════════"
echo "  TRIGGERING DATA PIPELINE"
echo "═══════════════════════════════════════════════════════════════"
echo "  Session ID: $SESSION_ID"
echo ""

# Find session directory
SESSION_PATH=$(python3 -c "
import sys
sys.path.insert(0, '$SCRIPT_DIR/../logging')
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

echo "  Session path: $SESSION_PATH"
echo ""

# Run pipeline at low priority
# nice +19 = lowest CPU priority
# ionice -c3 = idle I/O priority
echo "Starting pipeline (low priority)..."
nice -n 19 ionice -c3 python3 "$SCRIPT_DIR/process_session.py" "$SESSION_PATH" \
    > "$SESSION_PATH/metadata/pipeline.log" 2>&1 &

PIPELINE_PID=$!

echo "  Pipeline PID: $PIPELINE_PID"
echo "  Log: $SESSION_PATH/metadata/pipeline.log"
echo ""
echo "✓ Pipeline started in background"
echo "  Monitor with: tail -f $SESSION_PATH/metadata/pipeline.log"
echo "═══════════════════════════════════════════════════════════════"
