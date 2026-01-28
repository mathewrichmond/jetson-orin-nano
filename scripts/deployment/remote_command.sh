#!/bin/bash
# Execute commands on remote robot

set -e

TARGET_USER="${1:-nano}"
TARGET_HOST="${2:-localhost}"
shift 2
COMMAND="$@"

if [ -z "$COMMAND" ]; then
    echo "Usage: $0 <user> <host> <command>"
    echo ""
    echo "Examples:"
    echo "  $0 nano robot.local 'systemctl --user status isaac-robot.service'"
    echo "  $0 nano robot.local 'ros2 node list'"
    echo "  $0 nano robot.local 'df -h'"
    exit 1
fi

echo "═══════════════════════════════════════════════════════════════"
echo "  REMOTE COMMAND EXECUTION"
echo "═══════════════════════════════════════════════════════════════"
echo "  Target: $TARGET_USER@$TARGET_HOST"
echo "  Command: $COMMAND"
echo "───────────────────────────────────────────────────────────────"

# Log to audit file
AUDIT_LOG="/tmp/remote_commands_audit.log"
echo "$(date -Iseconds) | $TARGET_USER@$TARGET_HOST | $COMMAND" >> "$AUDIT_LOG"

# Execute command
if [ "$TARGET_HOST" = "localhost" ]; then
    eval "$COMMAND"
else
    ssh "$TARGET_USER@$TARGET_HOST" "$COMMAND"
fi

EXIT_CODE=$?

echo "───────────────────────────────────────────────────────────────"
echo "  Exit code: $EXIT_CODE"
echo "═══════════════════════════════════════════════════════════════"

exit $EXIT_CODE
