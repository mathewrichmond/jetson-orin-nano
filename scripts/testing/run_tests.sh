#!/bin/bash
# Test Runner Script
# Runs unit and integration tests locally

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

# Parse arguments
TEST_TYPE="${1:-all}"  # unit, integration, all
VERBOSE="${VERBOSE:-false}"
COVERAGE="${COVERAGE:-false}"

echo -e "${GREEN}=== Isaac Robot Test Runner ===${NC}"
echo "Test type: $TEST_TYPE"
echo "Repository: $REPO_ROOT"
echo ""

# Activate venv if it exists
if [ -f "$REPO_ROOT/.venv/bin/activate" ]; then
    source "$REPO_ROOT/.venv/bin/activate"
    echo "Activated Python virtual environment"
fi

# Install test dependencies if needed
if ! python3 -c "import pytest" 2>/dev/null; then
    echo "Installing test dependencies from pyproject.toml..."
    pip install -e ".[dev]"
fi

# Set test environment
export ROS_DOMAIN_ID=42
export MOCK_HARDWARE=true
export PYTHONPATH="$REPO_ROOT/src:$PYTHONPATH"

# Build pytest arguments
PYTEST_ARGS=()

if [ "$VERBOSE" = "true" ]; then
    PYTEST_ARGS+=("-v" "-s")
fi

if [ "$COVERAGE" = "true" ]; then
    PYTEST_ARGS+=("--cov=src" "--cov-report=html" "--cov-report=term")
fi

# Run tests based on type
case "$TEST_TYPE" in
    unit)
        echo -e "${GREEN}Running unit tests...${NC}"
        cd "$REPO_ROOT"
        pytest tests/unit/ "${PYTEST_ARGS[@]}" -m "unit"
        ;;
    
    integration)
        echo -e "${GREEN}Running integration tests...${NC}"
        
        # Check if ROS 2 is available
        if ! command -v ros2 &> /dev/null; then
            echo -e "${RED}Error: ROS 2 not found${NC}"
            echo "Source ROS 2: source /opt/ros/humble/setup.bash"
            exit 1
        fi
        
        # Check if workspace is built
        if [ ! -f "$REPO_ROOT/install/setup.bash" ]; then
            echo -e "${YELLOW}Warning: Workspace not built${NC}"
            echo "Building workspace..."
            
            source /opt/ros/humble/setup.bash
            cd "$REPO_ROOT"
            colcon build --symlink-install
        fi
        
        # Source workspace
        source /opt/ros/humble/setup.bash
        source "$REPO_ROOT/install/setup.bash"
        
        # Run integration tests
        cd "$REPO_ROOT"
        pytest tests/integration/ "${PYTEST_ARGS[@]}" -m "integration and not slow and not hardware"
        ;;
    
    all)
        echo -e "${GREEN}Running all tests...${NC}"
        
        # Unit tests first (faster)
        echo ""
        echo -e "${GREEN}1. Unit Tests${NC}"
        echo "------------------------"
        cd "$REPO_ROOT"
        pytest tests/unit/ "${PYTEST_ARGS[@]}" -m "unit"
        
        # Integration tests
        if command -v ros2 &> /dev/null; then
            echo ""
            echo -e "${GREEN}2. Integration Tests${NC}"
            echo "------------------------"
            
            if [ -f "$REPO_ROOT/install/setup.bash" ]; then
                source /opt/ros/humble/setup.bash
                source "$REPO_ROOT/install/setup.bash"
                
                cd "$REPO_ROOT"
                pytest tests/integration/ "${PYTEST_ARGS[@]}" -m "integration and not slow and not hardware"
            else
                echo -e "${YELLOW}Skipping integration tests (workspace not built)${NC}"
            fi
        else
            echo -e "${YELLOW}Skipping integration tests (ROS 2 not available)${NC}"
        fi
        ;;
    
    *)
        echo -e "${RED}Unknown test type: $TEST_TYPE${NC}"
        echo "Usage: $0 [unit|integration|all]"
        exit 1
        ;;
esac

echo ""
echo -e "${GREEN}=== Tests Complete ===${NC}"

# Show coverage report location if coverage was enabled
if [ "$COVERAGE" = "true" ]; then
    echo ""
    echo "Coverage report: $REPO_ROOT/htmlcov/index.html"
fi
