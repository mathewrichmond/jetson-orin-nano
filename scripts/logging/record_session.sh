#!/bin/bash
# Record ROS bag for a session with auto-rotation

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_DIR="$(cd "$SCRIPT_DIR/../../config/logging" && pwd)"

# Check if SESSION_ID is set
if [ -z "$SESSION_ID" ]; then
    echo "ERROR: SESSION_ID environment variable not set"
    echo "Create a session first with: python3 $SCRIPT_DIR/session_manager.py create"
    exit 1
fi

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
")

if [ -z "$SESSION_PATH" ]; then
    echo "ERROR: Session $SESSION_ID not found"
    exit 1
fi

BAG_DIR="$SESSION_PATH/raw"
BAG_PREFIX="$SESSION_ID"

echo "Starting ROS bag recording for session: $SESSION_ID"
echo "  Output directory: $BAG_DIR"
echo "  Bag prefix: $BAG_PREFIX"

# Load topics from configuration
TOPICS=$(python3 -c "
import yaml
with open('$CONFIG_DIR/session_structure.yaml') as f:
    config = yaml.safe_load(f)
    topics = config['session_structure']['topics']
    all_topics = []
    for category in topics.values():
        all_topics.extend(category)
    print(' '.join(all_topics))
")

echo "  Recording topics: $TOPICS"

# ROS bag recording parameters
SPLIT_SIZE_MB=1024  # 1GB
SPLIT_DURATION=600  # 10 minutes

# Source ROS environment
if [ -f "/home/nano/src/jetson-orin-nano/install/setup.bash" ]; then
    source /home/nano/src/jetson-orin-nano/install/setup.bash
fi

# Start recording with auto-rotation
# Using both size and duration limits
ros2 bag record \
    -o "$BAG_DIR/${BAG_PREFIX}_part" \
    --max-bag-size $((SPLIT_SIZE_MB * 1024 * 1024)) \
    --max-bag-duration $SPLIT_DURATION \
    --compression-mode file \
    --compression-format zstd \
    $TOPICS \
    > "$SESSION_PATH/metadata/bag_record.log" 2>&1 &

RECORD_PID=$!
echo $RECORD_PID > "$BAG_DIR/.record_pid"

echo "ROS bag recording started (PID: $RECORD_PID)"
echo "  Log: $SESSION_PATH/metadata/bag_record.log"
echo ""
echo "To stop recording: kill $RECORD_PID"
echo "Or use: $SCRIPT_DIR/stop_logging.sh"
