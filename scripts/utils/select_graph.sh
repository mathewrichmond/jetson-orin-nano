#!/bin/bash
# Select Robot Graph Configuration
# Usage: select_graph.sh [robot|monitor]
#   robot: Target/production graph (all robot nodes)
#   monitor: Viewer/logger graph (monitoring and visualization tools)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
GRAPH_CONFIG_FILE="${PROJECT_ROOT}/config/robot/selected_graph.txt"

# Get graph from argument or environment variable
GRAPH="${1:-${ROBOT_GRAPH:-minimal}}"

# Validate graph name (only robot or monitor)
case "$GRAPH" in
    robot|monitor)
        ;;
    *)
        echo "Error: Invalid graph '$GRAPH'"
        echo "Valid options: robot, monitor"
        exit 1
        ;;
esac

# Write to config file
echo "$GRAPH" > "$GRAPH_CONFIG_FILE"
echo "Selected graph: $GRAPH"
echo "Config file: $GRAPH_CONFIG_FILE"

# Show current selection
echo ""
echo "Current graph selection:"
cat "$GRAPH_CONFIG_FILE"
