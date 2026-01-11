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
  robot                Target/production graph (all robot nodes)

Examples:
  $0 start robot
  $0 status
  $0 select robot
  $0 verify

EOF
}

start_graph() {
    local graph="${1:-}"

    if [ -z "$graph" ]; then
        graph=$("${UTILS_DIR}/get_graph.sh" 2>/dev/null || echo "robot")
    fi

    # Select graph
    "${UTILS_DIR}/select_graph.sh" "$graph"

    # Start via systemd user service (no sudo required)
    if systemctl --user is-enabled isaac-robot.service &>/dev/null 2>&1; then
        echo "Starting robot system via systemd (graph: $graph)..."
        systemctl --user start isaac-robot.service
        systemctl --user status isaac-robot.service --no-pager -l || true
    else
        echo "Systemd service not installed. Starting directly (graph: $graph)..."
        echo "To install systemd service: ./scripts/system/setup_boot_service.sh"
        "${SCRIPT_DIR}/start_robot.sh"
    fi
}

stop_graph() {
    echo "Stopping robot system..."

    # Stop systemd user service if running (no sudo needed)
    if systemctl --user is-active --quiet isaac-robot.service 2>/dev/null; then
        echo "Stopping systemd service..."
        systemctl --user stop isaac-robot.service
    fi

    # Also stop any direct ROS 2 processes (in case started directly)
    pkill -f "ros2 launch" || true
    pkill -f "ros2 run" || true
    pkill -f "realsense_camera_node" || true
    pkill -f "system_monitor_node" || true
    pkill -f "usb_microphone_node" || true
    pkill -f "odrive_controller_node" || true
    pkill -f "irobot_serial_node" || true

    sleep 1
    echo "Robot system stopped"
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

    # Current graph selection
    echo "Current Graph Selection:"
    "${UTILS_DIR}/get_graph.sh" 2>/dev/null || echo "  (not set)"
    echo ""

    # Systemd user service status
    if systemctl --user is-enabled isaac-robot.service &>/dev/null 2>&1; then
        echo "Systemd Service:"
        systemctl --user status isaac-robot.service --no-pager -l || true
        echo ""
    fi

    # ROS 2 nodes
    if command -v ros2 &> /dev/null; then
        # Source ROS 2 if needed
        if [ -f "/opt/ros/humble/setup.bash" ]; then
            source /opt/ros/humble/setup.bash 2>/dev/null
        fi
        if [ -f ~/ros2_ws/install/setup.bash ]; then
            source ~/ros2_ws/install/setup.bash 2>/dev/null
        fi

        if ros2 node list &>/dev/null 2>&1; then
            echo "Running ROS 2 Nodes:"
            ros2 node list
            echo ""
            echo "Active Topics:"
            ros2 topic list 2>/dev/null | grep -v "^/parameter\|^/rosout" | head -20
        else
            echo "No ROS 2 nodes running"
        fi
    fi
}

select_graph() {
    local graph="${1:-robot}"

    if [ -z "$graph" ]; then
        graph="robot"
    fi

    if [ "$graph" != "robot" ]; then
        echo "Error: Only 'robot' graph is supported"
        exit 1
    fi

    "${UTILS_DIR}/select_graph.sh" "$graph"
    echo ""
    echo "Graph selected: $graph"
    echo "Restart service to apply: $0 restart $graph"
}

show_logs() {
    # Check if systemd user service is running
    if systemctl --user is-active --quiet isaac-robot.service 2>/dev/null; then
        echo "Viewing systemd service logs (Ctrl+C to exit)..."
        journalctl --user -u isaac-robot.service -f --no-pager
    elif pgrep -f "ros2 launch" > /dev/null 2>&1; then
        echo "Robot system is running directly (not via systemd)"
        echo "Logs are in: ~/.ros/log/"
        echo ""
        echo "To view systemd service logs (if installed):"
        echo "  journalctl --user -u isaac-robot.service -f"
    else
        echo "Robot system not running. Start with: $0 start"
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
