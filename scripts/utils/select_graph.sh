#!/bin/bash
# Select Robot Graph Configuration
# Usage: select_graph.sh [minimal|full|robot]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
GRAPH_CONFIG_FILE="${PROJECT_ROOT}/config/robot/selected_graph.txt"

# Get graph from argument or environment variable
GRAPH="${1:-${ROBOT_GRAPH:-minimal}}"

# Validate graph name
case "$GRAPH" in
    minimal|full|robot|bench_test)
        ;;
    *)
        echo "Error: Invalid graph '$GRAPH'"
        echo "Valid options: minimal, full, robot, bench_test"
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
