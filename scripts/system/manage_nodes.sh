#!/bin/bash
# Node Management Script
# Provides consistent tools for managing ROS 2 node runtimes
# Integrates with graph configuration and systemd

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UTILS_DIR="${SCRIPT_DIR}/../utils"

# Find Isaac root
ISAAC_ROOT="${ISAAC_ROOT:-$(python3 "${UTILS_DIR}/find_isaac_root.py" 2>/dev/null || echo "/opt/isaac-robot")}"

# Source ROS 2
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi

usage() {
    cat << EOF
Node Management Tool

Usage: $0 <command> [options]

Commands:
  start [graph] [group]    Start nodes from graph config
  stop                     Stop all running nodes
  status                   Show node status
  list                     List available nodes from graph
  restart [graph] [group]  Restart nodes
  logs [node]              Show logs for node(s)

Graph Options:
  graph: robot_graph.yaml, minimal_graph.yaml, full_graph.yaml (default: robot_graph.yaml)
  group: core, hardware, control, all (default: all)

Examples:
  $0 start robot_graph.yaml core
  $0 start full_graph.yaml all
  $0 status
  $0 logs system_monitor
  $0 stop

EOF
}

start_nodes() {
    local graph="${1:-robot_graph.yaml}"
    local group="${2:-all}"
    
    echo "=========================================="
    echo "Starting nodes from graph: $graph (group: $group)"
    echo "=========================================="
    
    cd "$ISAAC_ROOT"
    ros2 launch isaac_robot graph.launch.py \
        graph_config:="$graph" \
        group:="$group"
}

stop_nodes() {
    echo "=========================================="
    echo "Stopping all ROS 2 nodes"
    echo "=========================================="
    
    # Stop via systemd if running as service
    if systemctl is-active --quiet isaac-robot.service 2>/dev/null; then
        echo "Stopping isaac-robot.service..."
        sudo systemctl stop isaac-robot.service
    fi
    
    # Kill any remaining ROS 2 processes
    pkill -f "ros2 launch" || true
    pkill -f "ros2 run" || true
    
    # List any remaining nodes
    if ros2 node list &>/dev/null; then
        echo "Remaining nodes:"
        ros2 node list
    else
        echo "All nodes stopped"
    fi
}

show_status() {
    echo "=========================================="
    echo "Node Status"
    echo "=========================================="
    
    # Check systemd service
    if systemctl is-enabled isaac-robot.service &>/dev/null; then
        echo "Systemd Service:"
        systemctl status isaac-robot.service --no-pager -l || true
        echo ""
    fi
    
    # List ROS 2 nodes
    if ros2 node list &>/dev/null; then
        echo "Running ROS 2 Nodes:"
        ros2 node list
        echo ""
        echo "Node Details:"
        ros2 node list | while read node; do
            echo "  $node"
            ros2 node info "$node" 2>/dev/null | head -5 | sed 's/^/    /' || true
        done
    else
        echo "No ROS 2 nodes running"
    fi
    
    # Show topics
    echo ""
    echo "Active Topics:"
    if ros2 topic list &>/dev/null; then
        ros2 topic list | head -20
        local count=$(ros2 topic list 2>/dev/null | wc -l)
        if [ "$count" -gt 20 ]; then
            echo "  ... ($count total topics)"
        fi
    else
        echo "  No topics available"
    fi
}

list_nodes() {
    local graph="${1:-robot_graph.yaml}"
    
    echo "=========================================="
    echo "Available Nodes in Graph: $graph"
    echo "=========================================="
    
    # Try to load graph config
    local config_file="$ISAAC_ROOT/config/robot/$graph"
    if [ ! -f "$config_file" ]; then
        config_file="$ISAAC_ROOT/src/isaac_robot/config/robot/$graph"
    fi
    
    if [ ! -f "$config_file" ]; then
        echo "Error: Graph config not found: $graph"
        return 1
    fi
    
    echo "Nodes:"
    python3 << EOF
import yaml
with open('$config_file', 'r') as f:
    config = yaml.safe_load(f) or {}
    nodes = config.get('robot', {})
    for name, node_config in nodes.items():
        enabled = node_config.get('enabled', True)
        package = node_config.get('package', '?')
        executable = node_config.get('node', '?')
        status = '✓' if enabled else '✗'
        print(f"  {status} {name} ({package}/{executable})")
    
    print("\nGroups:")
    groups = config.get('groups', {})
    for group_name, node_list in groups.items():
        print(f"  {group_name}: {', '.join(node_list)}")
EOF
}

show_logs() {
    local node="${1:-}"
    
    if [ -n "$node" ]; then
        echo "Logs for node: $node"
        journalctl -u isaac-robot.service -f --no-pager | grep "$node" || true
    else
        echo "All node logs:"
        journalctl -u isaac-robot.service -f --no-pager || true
    fi
}

# Main command handling
case "${1:-}" in
    start)
        start_nodes "${2:-}" "${3:-}"
        ;;
    stop)
        stop_nodes
        ;;
    status)
        show_status
        ;;
    list)
        list_nodes "${2:-}"
        ;;
    restart)
        stop_nodes
        sleep 2
        start_nodes "${2:-}" "${3:-}"
        ;;
    logs)
        show_logs "${2:-}"
        ;;
    *)
        usage
        exit 1
        ;;
esac

