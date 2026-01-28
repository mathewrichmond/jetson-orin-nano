#!/bin/bash
# Run tests in Docker for CI consistency
# Usage: ./scripts/testing/docker_test.sh [lint|unit|all]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

TEST_TYPE="${1:-lint}"

# Docker image name
IMAGE_NAME="isaac-robot-ci:local"

echo -e "${GREEN}=== Docker CI Test Runner ===${NC}"
echo "Test type: $TEST_TYPE"
echo "Repository: $REPO_ROOT"
echo ""

# Check if Docker is available
if ! command -v docker &> /dev/null; then
    echo -e "${RED}Error: Docker not found${NC}"
    echo "Install Docker: https://docs.docker.com/get-docker/"
    exit 1
fi

# Build Docker image if it doesn't exist or if Dockerfile changed
if [ ! "$(docker images -q $IMAGE_NAME 2> /dev/null)" ] || \
   [ "$REPO_ROOT/Dockerfile.ci" -nt "$HOME/.cache/isaac-robot-ci-timestamp" ] || \
   [ "$REPO_ROOT/pyproject.toml" -nt "$HOME/.cache/isaac-robot-ci-timestamp" ]; then
    echo -e "${YELLOW}Building Docker image (first run or dependencies changed)...${NC}"
    docker build -f "$REPO_ROOT/Dockerfile.ci" -t "$IMAGE_NAME" "$REPO_ROOT"
    mkdir -p "$HOME/.cache"
    touch "$HOME/.cache/isaac-robot-ci-timestamp"
    echo ""
fi

# Run tests in Docker
case "$TEST_TYPE" in
    lint)
        echo -e "${GREEN}Running lint tests in Docker...${NC}"
        docker run --rm \
            -v "$REPO_ROOT:/workspace" \
            -w /workspace \
            "$IMAGE_NAME" \
            bash -c "
                echo '=== Black ===' && \
                black --check src/ tests/ && \
                echo '' && \
                echo '=== isort ===' && \
                isort --check-only src/ tests/ && \
                echo '' && \
                echo '=== flake8 ===' && \
                flake8 src/ tests/ --max-line-length=100 --extend-ignore=E203,W503 && \
                echo '' && \
                echo '✅ All lint checks passed!'
            "
        ;;
    
    unit)
        echo -e "${GREEN}Running unit tests in Docker...${NC}"
        docker run --rm \
            -v "$REPO_ROOT:/workspace" \
            -w /workspace \
            -e PYTHONPATH="/workspace/src:/workspace" \
            "$IMAGE_NAME" \
            bash -c "
                mkdir -p test-results && \
                pytest tests/unit/ -v -m unit --junit-xml=test-results/unit.xml
            "
        ;;
    
    all)
        echo -e "${GREEN}Running all pre-commit tests in Docker...${NC}"
        "$0" lint && "$0" unit
        ;;
    
    *)
        echo -e "${RED}Unknown test type: $TEST_TYPE${NC}"
        echo "Usage: $0 [lint|unit|all]"
        exit 1
        ;;
esac

echo ""
echo -e "${GREEN}=== Docker tests complete ===${NC}"
