#!/bin/bash
# Get Current Robot Graph Selection
# Returns: robot or monitor

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
GRAPH_CONFIG_FILE="${PROJECT_ROOT}/config/robot/selected_graph.txt"

# Priority: environment variable > config file > default
if [ -n "$ROBOT_GRAPH" ]; then
    echo "$ROBOT_GRAPH"
elif [ -f "$GRAPH_CONFIG_FILE" ]; then
    # Read first non-comment line
    grep -v '^#' "$GRAPH_CONFIG_FILE" | head -1 | tr -d '[:space:]'
else
    echo "robot"
fi
