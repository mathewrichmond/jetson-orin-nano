#!/bin/bash
# Unified Graph Management Script
# Manages robot graph runtime through systemd integration

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
UTILS_DIR="${PROJECT_ROOT}/scripts/utils"

usage() {
    cat << EOF
Unified Graph Management

Usage: $0 <command> [options]

Commands:
  start [graph]         Start robot system with graph (via systemd)
  stop                 Stop robot system
  restart [graph]      Restart robot system
  status               Show system status
  select [graph]       Select graph configuration
  logs                 Show system logs
  verify               Verify data streams from all sensors

Graph Options:
  minimal              Minimal system (system monitor only)
  full                 Full system (all components)
  robot                Default robot configuration
  bench_test           Bench test configuration (all hardware)

Examples:
  $0 start bench_test
  $0 status
  $0 select bench_test
  $0 verify

EOF
}

start_graph() {
    local graph="${1:-}"

    if [ -z "$graph" ]; then
        graph=$("${UTILS_DIR}/get_graph.sh" 2>/dev/null || echo "minimal")
    fi

    # Select graph
    "${UTILS_DIR}/select_graph.sh" "$graph"

    # Start via systemd if available, otherwise direct
    if systemctl is-enabled isaac-robot.service &>/dev/null 2>&1; then
        echo "Starting robot system via systemd (graph: $graph)..."
        sudo systemctl start isaac-robot.service
        sudo systemctl status isaac-robot.service --no-pager -l
    else
        echo "Starting robot system directly (graph: $graph)..."
        "${SCRIPT_DIR}/start_robot.sh"
    fi
}

stop_graph() {
    if systemctl is-active --quiet isaac-robot.service 2>/dev/null; then
        echo "Stopping robot system via systemd..."
        sudo systemctl stop isaac-robot.service
    else
        echo "Stopping robot system..."
        pkill -f "ros2 launch" || true
        pkill -f "ros2 run" || true
    fi
}

restart_graph() {
    local graph="${1:-}"
    stop_graph
    sleep 2
    start_graph "$graph"
}

show_status() {
    echo "=========================================="
    echo "Robot System Status"
    echo "=========================================="
    echo ""

    # Systemd service status
    if systemctl is-enabled isaac-robot.service &>/dev/null 2>&1; then
        echo "Systemd Service:"
        systemctl status isaac-robot.service --no-pager -l || true
        echo ""
    fi

    # Current graph selection
    echo "Current Graph Selection:"
    "${UTILS_DIR}/get_graph.sh" 2>/dev/null || echo "  (not set)"
    echo ""

    # ROS 2 nodes
    if command -v ros2 &> /dev/null; then
        if ros2 node list &>/dev/null 2>&1; then
            echo "Running ROS 2 Nodes:"
            ros2 node list
            echo ""
            echo "Active Topics:"
            ros2 topic list 2>/dev/null | head -20
        else
            echo "No ROS 2 nodes running"
        fi
    fi
}

select_graph() {
    local graph="${1:-}"

    if [ -z "$graph" ]; then
        echo "Error: Graph name required"
        echo "Valid options: minimal, full, robot, bench_test"
        exit 1
    fi

    "${UTILS_DIR}/select_graph.sh" "$graph"
    echo ""
    echo "Graph selected: $graph"
    echo "Restart service to apply: $0 restart $graph"
}

show_logs() {
    if systemctl is-active --quiet isaac-robot.service 2>/dev/null; then
        sudo journalctl -u isaac-robot.service -f --no-pager
    else
        echo "Service not running. Start with: $0 start"
    fi
}

verify_streams() {
    echo "=========================================="
    echo "Verifying Data Streams"
    echo "=========================================="
    echo ""

    if ! command -v ros2 &> /dev/null; then
        echo "Error: ROS 2 not available"
        exit 1
    fi

    # Source ROS 2
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
    fi

    if [ -f ~/ros2_ws/install/setup.bash ]; then
        source ~/ros2_ws/install/setup.bash
    fi

    # Check topics
    local all_passed=true

    check_topic() {
        local topic=$1
        local desc=$2

        echo -n "Checking $desc... "
        if ros2 topic list 2>/dev/null | grep -q "^$topic$"; then
            if timeout 2 ros2 topic echo "$topic" --once 2>/dev/null > /dev/null; then
                echo "✓"
            else
                echo "⚠ (no data)"
                all_passed=false
            fi
        else
            echo "✗ (not found)"
            all_passed=false
        fi
    }

    echo "System Monitor:"
    check_topic "/system/status" "System status"
    check_topic "/system/temperature/cpu" "CPU temperature"

    echo ""
    echo "RealSense Cameras:"
    check_topic "/realsense/status" "RealSense status"
    check_topic "/camera_front/color/image_raw" "Front camera"

    echo ""
    echo "USB Microphone:"
    check_topic "/microphone/status" "Microphone status"

    echo ""
    echo "ODrive Controller:"
    check_topic "/odrive/status" "ODrive status"
    check_topic "/odrive/imu" "ODrive IMU"

    echo ""
    echo "iRobot Serial:"
    check_topic "/irobot/status" "iRobot status"
    check_topic "/irobot/battery" "iRobot battery"

    echo ""
    if [ "$all_passed" = true ]; then
        echo "✓ All data streams verified"
    else
        echo "⚠ Some data streams need attention"
    fi
}

# Main command handling
case "${1:-}" in
    start)
        start_graph "${2:-}"
        ;;
    stop)
        stop_graph
        ;;
    restart)
        restart_graph "${2:-}"
        ;;
    status)
        show_status
        ;;
    select)
        select_graph "${2:-}"
        ;;
    logs)
        show_logs
        ;;
    verify)
        verify_streams
        ;;
    *)
        usage
        exit 1
        ;;
esac
