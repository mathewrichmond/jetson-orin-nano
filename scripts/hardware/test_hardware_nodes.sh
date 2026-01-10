#!/bin/bash
# Comprehensive Hardware Node Test Script
# Launches ROS 2 nodes for each hardware component and verifies they're working
# Tests: RealSense cameras, USB microphone, ODrive, iRobot serial

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Track test results
TESTS_PASSED=0
TESTS_FAILED=0
TEST_TIMEOUT=10  # seconds to wait for each test

# Source ROS 2
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

# Source workspace if it exists
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
fi

echo "=========================================="
echo "Isaac Robot Hardware Node Testing"
echo "Bench Setup - ROS 2 Node Verification"
echo "=========================================="
echo ""

# Function to print section header
print_section() {
    echo ""
    echo -e "${BLUE}=== $1 ===${NC}"
}

# Function to print success
print_success() {
    echo -e "${GREEN}✓ $1${NC}"
    ((TESTS_PASSED++))
}

# Function to print warning
print_warning() {
    echo -e "${YELLOW}⚠ $1${NC}"
    ((TESTS_FAILED++))
}

# Function to print error
print_error() {
    echo -e "${RED}✗ $1${NC}"
    ((TESTS_FAILED++))
}

# Function to check if topic exists and has data
check_topic() {
    local topic=$1
    local timeout=$2
    local expected_count=${3:-1}

    # Wait for topic to appear
    local count=0
    while [ $count -lt $timeout ]; do
        if ros2 topic list | grep -q "^${topic}$"; then
            # Check if topic has data
            local msg_count=$(timeout 2 ros2 topic echo --once ${topic} 2>/dev/null | wc -l || echo "0")
            if [ "$msg_count" -gt 0 ]; then
                return 0
            fi
        fi
        sleep 1
        ((count++))
    done
    return 1
}

# Function to test a node
test_node() {
    local node_name=$1
    local package=$2
    local executable=$3
    local topics=("${@:4}")
    local timeout=${TEST_TIMEOUT}

    print_section "Testing ${node_name}"

    # Check if package exists
    if ! ros2 pkg list | grep -q "^${package}$"; then
        print_error "${package} package not found (needs to be built)"
        return 1
    fi

    # Check if executable exists
    if ! ros2 pkg executables ${package} 2>/dev/null | grep -q "${executable}"; then
        print_error "${executable} executable not found"
        return 1
    fi

    print_success "Package and executable found"

    # Launch node in background
    echo "  Launching ${node_name}..."
    ros2 run ${package} ${executable} > /tmp/${node_name}_test.log 2>&1 &
    local node_pid=$!

    # Wait a bit for node to start
    sleep 2

    # Check if node is running
    if ! kill -0 $node_pid 2>/dev/null; then
        print_error "${node_name} failed to start"
        cat /tmp/${node_name}_test.log
        return 1
    fi

    print_success "${node_name} started (PID: $node_pid)"

    # Check for expected topics
    local topics_found=0
    for topic in "${topics[@]}"; do
        if check_topic "${topic}" 5; then
            print_success "Topic ${topic} is publishing"
            ((topics_found++))
        else
            print_warning "Topic ${topic} not found or not publishing"
        fi
    done

    # Stop the node
    kill $node_pid 2>/dev/null || true
    wait $node_pid 2>/dev/null || true

    if [ $topics_found -gt 0 ]; then
        print_success "${node_name} test completed (${topics_found}/${#topics[@]} topics verified)"
        return 0
    else
        print_error "${node_name} test failed (no topics publishing)"
        return 1
    fi
}

# ============================================
# 1. RealSense Camera Node Test
# ============================================
test_node \
    "RealSense Camera" \
    "realsense_camera" \
    "realsense_camera_node" \
    "/realsense/status" \
    "/camera_front/color/image_raw" \
    "/camera_rear/color/image_raw"

# ============================================
# 2. USB Microphone Node Test
# ============================================
test_node \
    "USB Microphone" \
    "usb_microphone" \
    "usb_microphone_node" \
    "/microphone/status"

# ============================================
# 3. ODrive Controller Node Test
# ============================================
test_node \
    "ODrive Controller" \
    "odrive_controller" \
    "odrive_controller_node" \
    "/odrive/status"

# ============================================
# 4. iRobot Serial Node Test
# ============================================
test_node \
    "iRobot Serial" \
    "irobot_serial" \
    "irobot_serial_node" \
    "/irobot/status"

# ============================================
# Summary
# ============================================
echo ""
echo "=========================================="
echo "Hardware Node Test Summary"
echo "=========================================="
echo -e "Tests Passed: ${GREEN}${TESTS_PASSED}${NC}"
echo -e "Tests Failed: ${RED}${TESTS_FAILED}${NC}"
echo ""

if [ $TESTS_FAILED -eq 0 ]; then
    echo -e "${GREEN}All hardware node tests passed!${NC}"
    echo ""
    echo "Next steps:"
    echo "  1. Launch full system: ros2 launch isaac_robot graph.launch.py graph_config:=robot_graph.yaml group:=bench_test"
    echo "  2. Monitor topics: ros2 topic list"
    echo "  3. View data: ros2 topic echo /<topic_name>"
    exit 0
else
    echo -e "${YELLOW}Some hardware node tests failed${NC}"
    echo ""
    echo "Troubleshooting:"
    echo "  1. Check hardware connections"
    echo "  2. Verify serial port permissions: ls -l /dev/ttyUSB*"
    echo "  3. Check logs: cat /tmp/<node_name>_test.log"
    echo "  4. Verify packages are built: colcon build --packages-select <package_name>"
    exit 1
fi
