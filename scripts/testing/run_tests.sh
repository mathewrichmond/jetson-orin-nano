#!/bin/bash
# Unified Test Runner
# Runs tests at different levels: lint, unit, integration, bench

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

usage() {
    cat << EOF
Unified Test Runner

Usage: $0 [level] [options]

Test Levels:
  lint          Code quality and consistency (fast, no hardware)
  unit          Hermetic mocked tests (fast, no hardware)
  integration   Node/graph level tests (medium, spoofed topics)
  bench         Real hardware tests (slow, requires hardware)
  all           Run all test levels (except bench by default)

Options:
  --bench       Include bench tests when running 'all'
  --verbose     Verbose output
  --coverage    Generate coverage report
  --package PKG Run tests for specific package only
  --help        Show this help

Examples:
  $0 lint                    # Run lint tests only
  $0 unit                    # Run unit tests only
  $0 integration             # Run integration tests
  $0 bench                   # Run bench tests (requires hardware)
  $0 all                     # Run lint, unit, integration
  $0 all --bench             # Run all including bench
  $0 unit --package realsense_camera

EOF
}

run_lint() {
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}Running Lint Tests${NC}"
    echo -e "${GREEN}========================================${NC}"

    cd "$REPO_ROOT"

    # Python linting
    echo "Running Black (format check)..."
    black --check --line-length 100 src/ tests/ || {
        echo -e "${YELLOW}Black found formatting issues. Run 'black src/ tests/' to fix.${NC}"
        return 1
    }

    echo "Running isort (import check)..."
    isort --check-only --profile black src/ tests/ || {
        echo -e "${YELLOW}isort found import issues. Run 'isort src/ tests/' to fix.${NC}"
        return 1
    }

    echo "Running flake8..."
    flake8 src/ tests/ --max-line-length=100 --extend-ignore=E203,W503 --exclude=.venv,venv,build,install,log || return 1

    echo "Running pylint..."
    pylint src/ --disable=C0111 || true  # Don't fail on missing docstrings

    echo "Running mypy..."
    mypy src/ --ignore-missing-imports || true

    # ROS 2 linting
    if command -v ament_lint &> /dev/null && [ -d ~/ros2_ws/src ]; then
        echo "Running ROS 2 ament_lint..."
        cd ~/ros2_ws
        source /opt/ros/humble/setup.bash 2>/dev/null || true
        colcon test --packages-select $(ls src/ 2>/dev/null | grep -v "^$" | head -5) --event-handlers console_direct+ 2>/dev/null || true
    fi

    echo -e "${GREEN}✓ Lint tests passed${NC}"
}

run_unit() {
    local package="${1:-}"
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}Running Unit Tests${NC}"
    echo -e "${GREEN}========================================${NC}"

    cd "$REPO_ROOT"

    local pytest_args="-v"
    if [ -n "$package" ]; then
        pytest_args="$pytest_args -k $package"
    fi

    pytest tests/unit/ $pytest_args || return 1

    echo -e "${GREEN}✓ Unit tests passed${NC}"
}

run_integration() {
    local package="${1:-}"
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}Running Integration Tests${NC}"
    echo -e "${GREEN}========================================${NC}"

    # Check ROS 2 is available
    if [ ! -f "/opt/ros/humble/setup.bash" ]; then
        echo -e "${YELLOW}Warning: ROS 2 not found. Skipping integration tests.${NC}"
        return 0
    fi

    source /opt/ros/humble/setup.bash

    if [ -f ~/ros2_ws/install/setup.bash ]; then
        source ~/ros2_ws/install/setup.bash
    fi

    cd "$REPO_ROOT"

    local pytest_args="-v -m integration"
    if [ -n "$package" ]; then
        pytest_args="$pytest_args -k $package"
    fi

    pytest tests/integration/ $pytest_args || return 1

    echo -e "${GREEN}✓ Integration tests passed${NC}"
}

run_bench() {
    local package="${1:-}"
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}Running Bench Tests${NC}"
    echo -e "${GREEN}========================================${NC}"
    echo -e "${YELLOW}Warning: Bench tests require real hardware!${NC}"

    # Check if we're on target hardware
    if [ ! -f "/etc/nv_tegra_release" ] && [ -z "$FORCE_BENCH" ]; then
        echo -e "${YELLOW}Skipping bench tests (not on Jetson hardware)${NC}"
        echo "Set FORCE_BENCH=1 to force run"
        return 0
    fi

    # Check ROS 2 is available
    if [ ! -f "/opt/ros/humble/setup.bash" ]; then
        echo -e "${RED}Error: ROS 2 not found. Cannot run bench tests.${NC}"
        return 1
    fi

    source /opt/ros/humble/setup.bash

    if [ -f ~/ros2_ws/install/setup.bash ]; then
        source ~/ros2_ws/install/setup.bash
    fi

    cd "$REPO_ROOT"

    local pytest_args="-v -m bench"
    if [ -n "$package" ]; then
        pytest_args="$pytest_args -k $package"
    fi

    pytest tests/bench/ $pytest_args || return 1

    echo -e "${GREEN}✓ Bench tests passed${NC}"
}

# Parse arguments
TEST_LEVEL=""
INCLUDE_BENCH=false
VERBOSE=false
COVERAGE=false
PACKAGE=""

while [[ $# -gt 0 ]]; do
    case $1 in
        lint|unit|integration|bench|all)
            TEST_LEVEL="$1"
            shift
            ;;
        --bench)
            INCLUDE_BENCH=true
            shift
            ;;
        --verbose|-v)
            VERBOSE=true
            shift
            ;;
        --coverage|-c)
            COVERAGE=true
            shift
            ;;
        --package|-p)
            PACKAGE="$2"
            shift 2
            ;;
        --help|-h)
            usage
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            usage
            exit 1
            ;;
    esac
done

if [ -z "$TEST_LEVEL" ]; then
    echo "Error: Test level required"
    usage
    exit 1
fi

# Run tests
FAILED=0

case "$TEST_LEVEL" in
    lint)
        run_lint || FAILED=1
        ;;
    unit)
        run_unit "$PACKAGE" || FAILED=1
        ;;
    integration)
        run_integration "$PACKAGE" || FAILED=1
        ;;
    bench)
        run_bench "$PACKAGE" || FAILED=1
        ;;
    all)
        run_lint || FAILED=1
        run_unit "$PACKAGE" || FAILED=1
        run_integration "$PACKAGE" || FAILED=1
        if [ "$INCLUDE_BENCH" = true ]; then
            run_bench "$PACKAGE" || FAILED=1
        fi
        ;;
esac

if [ $FAILED -eq 0 ]; then
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}All tests passed!${NC}"
    echo -e "${GREEN}========================================${NC}"
    exit 0
else
    echo -e "${RED}========================================${NC}"
    echo -e "${RED}Some tests failed${NC}"
    echo -e "${RED}========================================${NC}"
    exit 1
fi
