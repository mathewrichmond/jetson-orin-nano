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
  node-list            List all available nodes from graph config
  node-start <node>    Start a single node (without restarting entire graph)
  node-stop <node>     Stop a single node
  node-restart <node>  Restart a single node
  container-restart    Restart composable container (for debugging composable nodes)
  container-logs [node] Show/filter logs from composable container (optionally filter by node)
                      Use '--no-follow' as second arg to show recent logs without following
  container-status     Show status of nodes in composable container
  container-perf       Monitor performance (CPU/memory) of composable container

Graph Options:
  robot                Target/production graph (all robot nodes)

Examples:
  $0 start robot
  $0 status
  $0 select robot
  $0 verify
  $0 node-list
  $0 node-start realsense_camera
  $0 node-restart realsense_camera
  $0 container-restart
  $0 container-logs realsense_camera
  $0 container-status
  $0 container-perf

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

source_ros2() {
    # Source ROS 2 if available
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash 2>/dev/null || true
    fi
    if [ -f ~/ros2_ws/install/setup.bash ]; then
        source ~/ros2_ws/install/setup.bash 2>/dev/null || true
    fi
}

list_nodes() {
    local graph="${1:-}"
    
    if [ -z "$graph" ]; then
        graph=$("${UTILS_DIR}/get_graph.sh" 2>/dev/null || echo "robot")
    fi

    echo "=========================================="
    echo "Available Nodes (graph: $graph)"
    echo "=========================================="
    echo ""

    local graph_config="${PROJECT_ROOT}/config/robot/${graph}_graph.yaml"
    if [ ! -f "$graph_config" ]; then
        echo "Error: Graph config not found: $graph_config" >&2
        exit 1
    fi

    # Use Python to parse YAML and list nodes
    python3 << EOF
import yaml
import sys

try:
    with open("$graph_config", "r") as f:
        config = yaml.safe_load(f) or {}
    
    nodes_config = config.get("robot", {})
    
    enabled_nodes = []
    disabled_nodes = []
    
    for node_name, node_config in nodes_config.items():
        if not isinstance(node_config, dict) or "package" not in node_config:
            continue
        
        if node_config.get("enabled", True):
            enabled_nodes.append((node_name, node_config))
        else:
            disabled_nodes.append((node_name, node_config))
    
    if enabled_nodes:
        print("Enabled Nodes:")
        for node_name, node_config in sorted(enabled_nodes):
            package = node_config.get("package", "?")
            executable = node_config.get("node", "?")
            namespace = node_config.get("namespace", "")
            ns_str = f" (ns: {namespace})" if namespace else ""
            print(f"  {node_name:30} {package}/{executable}{ns_str}")
        print("")
    
    if disabled_nodes:
        print("Disabled Nodes:")
        for node_name, node_config in sorted(disabled_nodes):
            print(f"  {node_name:30} (disabled)")
        print("")
    
    print(f"Total: {len(enabled_nodes)} enabled, {len(disabled_nodes)} disabled")
    
except Exception as e:
    print(f"Error parsing graph config: {e}", file=sys.stderr)
    sys.exit(1)
EOF
}

start_node() {
    local node_name="${1:-}"
    
    if [ -z "$node_name" ]; then
        echo "Error: Node name required" >&2
        echo "Usage: $0 node-start <node_name>" >&2
        echo "Use '$0 node-list' to see available nodes" >&2
        exit 1
    fi

    # Get graph selection
    local graph=$("${UTILS_DIR}/get_graph.sh" 2>/dev/null || echo "robot")

    echo "=========================================="
    echo "Starting Node: $node_name"
    echo "=========================================="
    echo ""

    # Get node configuration
    local node_config
    node_config=$("${UTILS_DIR}/get_node_config.sh" "$node_name" "$graph" 2>&1)
    if [ $? -ne 0 ]; then
        echo "$node_config" >&2
        exit 1
    fi

    # Parse node config (JSON)
    local package executable namespace params_json
    package=$(echo "$node_config" | python3 -c "import sys, json; print(json.load(sys.stdin)['package'])" 2>/dev/null)
    executable=$(echo "$node_config" | python3 -c "import sys, json; print(json.load(sys.stdin)['executable'])" 2>/dev/null)
    namespace=$(echo "$node_config" | python3 -c "import sys, json; print(json.load(sys.stdin).get('namespace', ''))" 2>/dev/null)
    params_json=$(echo "$node_config" | python3 -c "import sys, json; print(json.dumps(json.load(sys.stdin)['parameters']))" 2>/dev/null)

    if [ -z "$package" ] || [ -z "$executable" ]; then
        echo "Error: Could not extract node configuration" >&2
        exit 1
    fi

    # Check if node is already running
    source_ros2
    if ros2 node list 2>/dev/null | grep -q "$node_name"; then
        echo "Node '$node_name' is already running"
        echo "Use '$0 node-restart $node_name' to restart it"
        exit 0
    fi

    # Check if node is in composable container
    if ros2 node list 2>/dev/null | grep -q "composable_container"; then
        local composable_nodes="realsense_camera nvblox_processor sensor_fusion"
        if echo "$composable_nodes" | grep -q "$node_name"; then
            echo "Warning: Node '$node_name' is part of the composable container"
            echo "Composable nodes cannot be started individually while the container is running"
            echo "Stop the graph first, or restart the entire graph"
            exit 1
        fi
    fi

    echo "Package: $package"
    echo "Executable: $executable"
    [ -n "$namespace" ] && echo "Namespace: $namespace"
    echo ""

    # Source ROS 2
    source_ros2

    # Build parameter arguments
    local param_args=""
    if [ -n "$params_json" ] && [ "$params_json" != "{}" ]; then
        # Check if we need a YAML file for complex nested structures
        local has_nested_dict
        has_nested_dict=$(echo "$params_json" | python3 -c "import sys, json; params=json.load(sys.stdin); print('true' if any(isinstance(v, dict) for v in params.values()) else 'false')" 2>/dev/null || echo "false")
        
        if [ "$has_nested_dict" = "true" ]; then
            # Use YAML file for complex parameters
            local temp_param_file="/tmp/${node_name}_params.yaml"
            echo "$params_json" | python3 << PYEOF > "$temp_param_file"
import sys
import json
import yaml

params = json.load(sys.stdin)
with open("$temp_param_file", 'w') as f:
    yaml.dump(params, f, default_flow_style=False)
PYEOF
            param_args="--ros-args --params-file $temp_param_file"
        else
            # Convert JSON parameters to ROS 2 parameter format
            param_args=$(echo "$params_json" | python3 << 'PYEOF'
import sys
import json

params = json.load(sys.stdin)
param_list = []
for key, value in params.items():
    if isinstance(value, bool):
        param_list.append(f"{key}:={str(value).lower()}")
    elif isinstance(value, (int, float)):
        param_list.append(f"{key}:={value}")
    elif isinstance(value, str):
        # Quote strings, escape internal quotes
        escaped_value = value.replace('"', '\\"')
        param_list.append(f'{key}:="{escaped_value}"')
    elif isinstance(value, list):
        # Format arrays - quote string elements
        if value and isinstance(value[0], str):
            formatted_items = ','.join(f'"{item}"' for item in value)
            param_list.append(f"{key}:=[{formatted_items}]")
        else:
            param_list.append(f"{key}:=[{','.join(str(v) for v in value)}]")
print(' '.join(f"--ros-args -p {p}" for p in param_list))
PYEOF
            )
        fi
    fi

    # Build namespace argument
    local ns_arg=""
    if [ -n "$namespace" ]; then
        ns_arg="--ros-args -r __ns:=$namespace"
    fi

    # Start node in background
    echo "Starting node..."
    cd "$PROJECT_ROOT"
    
    # Run node with proper arguments
    nohup ros2 run "$package" "$executable" \
        --ros-args -r __node:="$node_name" \
        $ns_arg \
        $param_args \
        > "/tmp/${node_name}_node.log" 2>&1 &
    
    local pid=$!
    echo "Node started with PID: $pid"
    echo "Logs: /tmp/${node_name}_node.log"
    echo ""
    echo "To check if it's running: ros2 node list | grep $node_name"
    echo "To stop it: $0 node-stop $node_name"
}

stop_node() {
    local node_name="${1:-}"
    
    if [ -z "$node_name" ]; then
        echo "Error: Node name required" >&2
        echo "Usage: $0 node-stop <node_name>" >&2
        exit 1
    fi

    echo "=========================================="
    echo "Stopping Node: $node_name"
    echo "=========================================="
    echo ""

    source_ros2

    # Check if node is running
    if ! ros2 node list 2>/dev/null | grep -q "$node_name"; then
        echo "Node '$node_name' is not running"
        exit 0
    fi

    # Check if node is in composable container
    if ros2 node list 2>/dev/null | grep -q "composable_container"; then
        local composable_nodes="realsense_camera nvblox_processor sensor_fusion"
        if echo "$composable_nodes" | grep -q "$node_name"; then
            echo "Warning: Node '$node_name' is part of the composable container"
            echo "Cannot stop individual composable nodes - they run in a shared container"
            echo "Restart the entire graph to restart this node"
            exit 1
        fi
    fi

    # Find and kill the process
    # Try multiple methods to find the process
    local pids=""
    
    # Method 1: Find by executable name pattern
    local executable_pattern="${node_name}_node"
    pids=$(pgrep -f "$executable_pattern" 2>/dev/null || true)
    
    # Method 2: Find by ROS 2 node name
    if [ -z "$pids" ]; then
        # Get node info to find process
        local node_info
        node_info=$(ros2 node info "/${node_name}" 2>/dev/null || true)
        # This is tricky - ROS 2 doesn't directly expose PIDs
        # We'll use process name matching instead
    fi
    
    # Method 3: Find by package/executable combination
    if [ -z "$pids" ]; then
        # Get node config to find executable name
        local graph=$("${UTILS_DIR}/get_graph.sh" 2>/dev/null || echo "robot")
        local node_config
        node_config=$("${UTILS_DIR}/get_node_config.sh" "$node_name" "$graph" 2>/dev/null || echo "")
        if [ -n "$node_config" ]; then
            local executable
            executable=$(echo "$node_config" | python3 -c "import sys, json; print(json.load(sys.stdin)['executable'])" 2>/dev/null || echo "")
            if [ -n "$executable" ]; then
                pids=$(pgrep -f "$executable" 2>/dev/null || true)
            fi
        fi
    fi

    if [ -z "$pids" ]; then
        echo "Warning: Could not find process for node '$node_name'"
        echo "Node may have already stopped, or it's running in a container"
        exit 0
    fi

    # Kill the process(es)
    echo "Found process(es): $pids"
    for pid in $pids; do
        echo "Stopping process $pid..."
        kill "$pid" 2>/dev/null || true
    done

    # Wait a bit and force kill if still running
    sleep 1
    for pid in $pids; do
        if kill -0 "$pid" 2>/dev/null; then
            echo "Force killing process $pid..."
            kill -9 "$pid" 2>/dev/null || true
        fi
    done

    sleep 1
    echo "Node '$node_name' stopped"
}

restart_node() {
    local node_name="${1:-}"
    
    if [ -z "$node_name" ]; then
        echo "Error: Node name required" >&2
        echo "Usage: $0 node-restart <node_name>" >&2
        exit 1
    fi

    echo "Restarting node: $node_name"
    echo ""
    
    # Stop the node (ignore errors if not running)
    stop_node "$node_name" 2>/dev/null || true
    
    # Wait a moment
    sleep 2
    
    # Start the node
    start_node "$node_name"
}

restart_container() {
    echo "=========================================="
    echo "Restarting Composable Container"
    echo "=========================================="
    echo ""

    source_ros2

    # Check if container is running
    if ! ros2 node list 2>/dev/null | grep -q "composable_container"; then
        echo "Composable container is not running"
        echo "Start the graph first: $0 start robot"
        exit 1
    fi

    # Find the container process
    local container_pid
    container_pid=$(pgrep -f "composable_container" | head -1)

    if [ -z "$container_pid" ]; then
        echo "Warning: Could not find composable container process"
        echo "Container node exists but process not found - may need full graph restart"
        exit 1
    fi

    echo "Found container process: $container_pid"
    echo "Restarting container..."

    # Stop the container gracefully
    kill -SIGINT "$container_pid" 2>/dev/null || true
    sleep 2

    # Force kill if still running
    if kill -0 "$container_pid" 2>/dev/null; then
        echo "Force stopping container..."
        kill -9 "$container_pid" 2>/dev/null || true
        sleep 1
    fi

    # Wait for container to fully stop
    sleep 2

    # Check if systemd service is managing it
    if systemctl --user is-active --quiet isaac-robot.service 2>/dev/null; then
        echo "Container is managed by systemd service"
        echo "Restarting systemd service..."
        systemctl --user restart isaac-robot.service
        echo "Container restarted via systemd"
    else
        echo "Warning: Container was not started via systemd"
        echo "You may need to restart the entire graph: $0 restart robot"
    fi
}

show_container_logs() {
    local filter_node="${1:-}"
    local follow="${2:-true}"  # Default to following logs

    echo "=========================================="
    if [ -n "$filter_node" ]; then
        echo "Container Logs (filtered: $filter_node)"
    else
        echo "Container Logs"
    fi
    echo "=========================================="
    echo ""

    # Check if systemd service is running
    if systemctl --user is-active --quiet isaac-robot.service 2>/dev/null; then
        local journal_cmd="journalctl --user -u isaac-robot.service --no-pager"
        
        # Add follow flag if requested
        if [ "$follow" = "true" ]; then
            journal_cmd="$journal_cmd -f"
            if [ -n "$filter_node" ]; then
                echo "Filtering logs for node: $filter_node (Press Ctrl+C to exit)"
            else
                echo "Showing all container logs (Press Ctrl+C to exit)..."
            fi
        else
            # Show last 100 lines
            journal_cmd="$journal_cmd -n 100"
            if [ -n "$filter_node" ]; then
                echo "Showing last 100 log lines filtered for: $filter_node"
            else
                echo "Showing last 100 log lines"
            fi
        fi
        echo ""

        if [ -n "$filter_node" ]; then
            # Filter by node name (case-insensitive)
            $journal_cmd | grep --line-buffered -i "$filter_node" || {
                echo "No logs found for '$filter_node'"
                echo "Showing recent container logs instead:"
                $journal_cmd | tail -20
            }
        else
            $journal_cmd
        fi
    else
        # Try to find container process
        local container_pid
        container_pid=$(pgrep -f "composable_container" | head -1)

        if [ -z "$container_pid" ]; then
            echo "Container is not running"
            exit 1
        fi

        echo "Container PID: $container_pid"
        echo ""
        
        # Try to find ROS log files
        local ros_log_dir="$HOME/.ros/log"
        if [ -d "$ros_log_dir" ]; then
            local latest_log
            latest_log=$(find "$ros_log_dir" -name "*composable_container*" -type f -printf '%T@ %p\n' 2>/dev/null | sort -n | tail -1 | cut -d' ' -f2-)
            
            if [ -n "$latest_log" ] && [ -f "$latest_log" ]; then
                echo "Found log file: $latest_log"
                echo ""
                if [ -n "$filter_node" ]; then
                    tail -100 "$latest_log" | grep -i "$filter_node" || tail -20 "$latest_log"
                else
                    tail -100 "$latest_log"
                fi
            else
                echo "Note: Logs may be in systemd journal or ROS log directory"
                echo ""
                echo "To view systemd logs: journalctl --user -u isaac-robot.service -f"
                echo "To view ROS logs: ls -lt ~/.ros/log/latest/"
            fi
        else
            echo "Note: Logs may be in systemd journal"
            echo "To view: journalctl --user -u isaac-robot.service -f"
        fi
    fi
}

show_container_status() {
    echo "=========================================="
    echo "Composable Container Status"
    echo "=========================================="
    echo ""

    source_ros2

    # Check if container is running
    if ! ros2 node list 2>/dev/null | grep -q "composable_container"; then
        echo "Composable container is not running"
        exit 0
    fi

    # Get container process info
    local container_pid
    container_pid=$(pgrep -f "composable_container" | head -1)

    if [ -n "$container_pid" ]; then
        echo "Container Process:"
        echo "  PID: $container_pid"
        
        # Get process stats
        if command -v ps > /dev/null; then
            local cpu_mem
            cpu_mem=$(ps -p "$container_pid" -o %cpu,%mem,rss,vsz --no-headers 2>/dev/null || echo "")
            if [ -n "$cpu_mem" ]; then
                echo "  CPU: $(echo $cpu_mem | awk '{print $1}')%"
                echo "  Memory: $(echo $cpu_mem | awk '{print $2}')% ($(echo $cpu_mem | awk '{printf "%.1f", $3/1024}') MB)"
            fi
        fi
        echo ""
    fi

    # List nodes in container
    echo "Nodes in Container:"
    local container_node
    container_node=$(ros2 node list 2>/dev/null | grep composable_container | head -1)
    
    if [ -n "$container_node" ]; then
        echo "  Container node: $container_node"
        echo ""
        
        # Try to get node info
        if ros2 node info "$container_node" &>/dev/null; then
            echo "Node Details:"
            ros2 node info "$container_node" 2>/dev/null | head -20 | sed 's/^/  /'
        fi
    fi

    echo ""
    echo "Composable Nodes Status:"
    local composable_nodes="realsense_camera nvblox_processor sensor_fusion"
    local running_nodes
    running_nodes=$(ros2 node list 2>/dev/null || echo "")
    
    for node_name in $composable_nodes; do
        # Check if node appears in ROS 2 node list (may be namespaced)
        if echo "$running_nodes" | grep -q "$node_name"; then
            echo "  $node_name: ✓ running"
        else
            echo "  $node_name: ⚠ not detected in node list"
        fi
    done

    echo ""
    echo "Topics Published by Container Nodes:"
    if command -v ros2 > /dev/null; then
        local topics
        topics=$(ros2 topic list 2>/dev/null | grep -E "(realsense|nvblox|sensor_fusion|camera_front|camera_rear)" | head -15)
        if [ -n "$topics" ]; then
            echo "$topics" | sed 's/^/  /'
        else
            echo "  (none found)"
        fi
    fi
}

monitor_container_perf() {
    echo "=========================================="
    echo "Container Performance Monitoring"
    echo "=========================================="
    echo "Press Ctrl+C to exit"
    echo ""

    source_ros2

    # Find container process
    local container_pid
    container_pid=$(pgrep -f "composable_container" | head -1)

    if [ -z "$container_pid" ]; then
        echo "Error: Composable container is not running"
        exit 1
    fi

    echo "Monitoring container process: $container_pid"
    echo ""

    # Monitor loop
    while true; do
        if ! kill -0 "$container_pid" 2>/dev/null; then
            echo "Container process has stopped"
            break
        fi

        # Get process stats
        if command -v ps > /dev/null; then
            local stats
            stats=$(ps -p "$container_pid" -o %cpu,%mem,rss,vsz,etime --no-headers 2>/dev/null)
            if [ -n "$stats" ]; then
                local cpu mem rss vsz etime
                cpu=$(echo $stats | awk '{print $1}')
                mem=$(echo $stats | awk '{print $2}')
                rss=$(echo $stats | awk '{printf "%.1f", $3/1024}')
                vsz=$(echo $stats | awk '{printf "%.1f", $4/1024}')
                etime=$(echo $stats | awk '{print $5}')
                
                # Clear line and print stats
                printf "\rCPU: %5s%%  Memory: %5s%% (%6s MB)  RSS: %6s MB  VSZ: %7s MB  Uptime: %s" \
                    "$cpu" "$mem" "$rss" "$rss" "$vsz" "$etime"
            fi
        fi

        sleep 1
    done
    echo ""
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
    node-list)
        list_nodes "${2:-}"
        ;;
    node-start)
        start_node "${2:-}"
        ;;
    node-stop)
        stop_node "${2:-}"
        ;;
    node-restart)
        restart_node "${2:-}"
        ;;
    container-restart)
        restart_container
        ;;
    container-logs)
        # Handle --no-follow flag
        if [ "${3:-}" = "--no-follow" ]; then
            show_container_logs "${2:-}" "false"
        else
            show_container_logs "${2:-}" "true"
        fi
        ;;
    container-status)
        show_container_status
        ;;
    container-perf)
        monitor_container_perf
        ;;
    *)
        usage
        exit 1
        ;;
esac
