#!/bin/bash
# Get Node Configuration from Graph
# Extracts node configuration (package, executable, namespace, parameters) from graph YAML
# Usage: get_node_config.sh <node_name> [graph]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

NODE_NAME="${1:-}"
GRAPH="${2:-}"

if [ -z "$NODE_NAME" ]; then
    echo "Error: Node name required" >&2
    exit 1
fi

# Get graph selection
if [ -z "$GRAPH" ]; then
    GRAPH=$("${SCRIPT_DIR}/get_graph.sh" 2>/dev/null || echo "robot")
fi

# Find config file
GRAPH_CONFIG="${PROJECT_ROOT}/config/robot/${GRAPH}_graph.yaml"

if [ ! -f "$GRAPH_CONFIG" ]; then
    echo "Error: Graph config not found: $GRAPH_CONFIG" >&2
    exit 1
fi

# Use Python to parse YAML and extract node config
python3 << EOF
import yaml
import json
import sys

try:
    with open("$GRAPH_CONFIG", "r") as f:
        config = yaml.safe_load(f) or {}
    
    nodes_config = config.get("robot", {})
    node_config = nodes_config.get("$NODE_NAME", {})
    
    if not node_config:
        print(f"Error: Node '$NODE_NAME' not found in graph config", file=sys.stderr)
        sys.exit(1)
    
    if not node_config.get("enabled", True):
        print(f"Error: Node '$NODE_NAME' is disabled in graph config", file=sys.stderr)
        sys.exit(1)
    
    # Extract node configuration
    result = {
        "package": node_config.get("package", ""),
        "executable": node_config.get("node", ""),
        "namespace": node_config.get("namespace", ""),
        "parameters": node_config.get("parameters", {}),
        "enabled": node_config.get("enabled", True),
    }
    
    # Output as JSON for easy parsing
    print(json.dumps(result))
    
except Exception as e:
    print(f"Error parsing graph config: {e}", file=sys.stderr)
    sys.exit(1)
EOF
