# Unified Testing Framework

This guide covers the unified testing system for the Isaac robot, providing consistent testing across all packages and modules.

## Overview

The testing framework has **four levels**:

1. **Lint Tests** - Code quality and consistency (fast, no hardware)
2. **Unit Tests** - Hermetic mocked tests (fast, no hardware)
3. **Integration Tests** - Node/graph level tests (medium, spoofed topics)
4. **Bench Tests** - Real hardware tests (slow, requires hardware)

All tests use the same infrastructure and can be run consistently across:
- Pre-commit hooks (local development)
- GitHub CI (automated)
- Target hardware (manual/CI)

## Quick Start

### Run All Tests (except bench)

```bash
./scripts/testing/run_tests.sh all
```

### Run Specific Test Level

```bash
# Lint only
./scripts/testing/run_tests.sh lint

# Unit tests only
./scripts/testing/run_tests.sh unit

# Integration tests
./scripts/testing/run_tests.sh integration

# Bench tests (requires hardware)
./scripts/testing/run_tests.sh bench
```

### Run Tests for Specific Package

```bash
./scripts/testing/run_tests.sh unit --package realsense_camera
```

## Test Levels

### 1. Lint Tests

**Purpose**: Code quality and consistency

**What they check**:
- Code formatting (Black)
- Import sorting (isort)
- Style violations (flake8)
- Type checking (mypy)
- Security issues (bandit)
- ROS 2 linting (ament_lint)

**Run**:
```bash
./scripts/testing/run_tests.sh lint
```

**Pre-commit**: Runs automatically on commit

**CI**: Runs on every push/PR

### 2. Unit Tests

**Purpose**: Hermetic, mocked tests that don't require ROS 2 or hardware

**Location**: `tests/unit/`

**Characteristics**:
- Fast execution
- No external dependencies
- Mocked ROS 2 interfaces
- Test individual functions/classes

**Example**:
```python
@pytest.mark.unit
class TestGraphManager:
    def test_load_config(self):
        # Test with mocked file system
        pass
```

**Run**:
```bash
./scripts/testing/run_tests.sh unit
```

**Pre-commit**: Runs on commit (can be disabled for speed)

**CI**: Runs on every push/PR

### 3. Integration Tests

**Purpose**: Node/graph level tests with spoofed topics and synthetic data

**Location**: `tests/integration/`

**Characteristics**:
- Requires ROS 2 (but not hardware)
- Uses spoofed/synthetic data
- Tests node interactions
- Tests graph configurations
- Can use recorded bag files

**Example**:
```python
@pytest.mark.integration
class TestNodeLaunch:
    def test_nodes_can_be_launched(self):
        # Launch nodes and verify they start
        pass
```

**Run**:
```bash
./scripts/testing/run_tests.sh integration
```

**CI**: Runs on every push/PR (if ROS 2 available)

### 4. Bench Tests

**Purpose**: Real hardware tests on target (Jetson)

**Location**: `tests/bench/`

**Characteristics**:
- Requires actual hardware
- Tests real sensors/actuators
- Can be slow
- May require manual verification
- Should be run before releases

**Example**:
```python
@pytest.mark.bench
@pytest.mark.hardware
class TestCameraVisualization:
    def test_camera_node_starts(self):
        # Test with real cameras
        pass
```

**Run**:
```bash
# On Jetson
./scripts/testing/run_tests.sh bench

# Or use bench test scripts
./scripts/testing/bench_test_cameras.sh
```

**CI**: Manual trigger only (workflow_dispatch)

## Test Organization

```
tests/
├── __init__.py
├── conftest.py              # Shared fixtures
├── unit/                    # Unit tests
│   ├── __init__.py
│   └── test_*.py
├── integration/            # Integration tests
│   ├── __init__.py
│   └── test_*.py
└── bench/                   # Bench tests
    ├── __init__.py
    └── test_*.py
```

## Writing Tests

### Unit Test Example

```python
import pytest
from unittest.mock import Mock, patch

@pytest.mark.unit
class TestMyNode:
    def test_initialization(self):
        # Mock ROS 2 dependencies
        with patch('rclpy.init'):
            node = MyNode()
            assert node is not None
```

### Integration Test Example

```python
import pytest
import rclpy

@pytest.mark.integration
class TestMyNodeIntegration:
    @pytest.fixture(autouse=True)
    def setup_ros2(self):
        rclpy.init()
        yield
        rclpy.shutdown()

    def test_node_publishes(self):
        # Launch node and verify topics
        pass
```

### Bench Test Example

```python
import pytest

@pytest.mark.bench
@pytest.mark.hardware
class TestCameraBench:
    def test_camera_visualization(self):
        # Test with real hardware
        # May require manual verification
        pass
```

## Pytest Markers

Tests are marked with pytest markers:

- `@pytest.mark.lint` - Lint tests
- `@pytest.mark.unit` - Unit tests
- `@pytest.mark.integration` - Integration tests
- `@pytest.mark.bench` - Bench tests
- `@pytest.mark.hardware` - Requires hardware

Run tests by marker:
```bash
pytest -m unit
pytest -m integration
pytest -m bench
```

## Pre-commit Hooks

Pre-commit hooks run automatically on commit:

```bash
# Install hooks
pre-commit install

# Run manually
pre-commit run --all-files

# Run specific hook
pre-commit run lint --all-files
```

Hooks run:
- Lint tests (formatting, style)
- Unit tests (optional, can disable for speed)

## GitHub CI

CI runs automatically on push/PR:

1. **Lint** - Always runs
2. **Unit** - Always runs
3. **Integration** - Runs if ROS 2 available
4. **Bench** - Manual trigger only (workflow_dispatch)

View CI status: `.github/workflows/ci.yml`

## Bench Test Scripts

Dedicated scripts for common bench tests:

```bash
# Camera visualization bench test
./scripts/testing/bench_test_cameras.sh

# Or use the visualization script
./scripts/visualization/bench_test_cameras.sh
```

## Test Coverage

Coverage reports are generated:

```bash
# Generate HTML report
./scripts/testing/run_tests.sh unit --coverage

# View report
open htmlcov/index.html
```

## Best Practices

1. **Write unit tests first** - Fast feedback
2. **Mock external dependencies** - Keep tests hermetic
3. **Use integration tests for node interactions** - Test graph behavior
4. **Use bench tests sparingly** - Only for hardware validation
5. **Mark tests appropriately** - Use correct pytest markers
6. **Keep tests fast** - Unit < 1s, Integration < 10s
7. **Document test requirements** - Note hardware needs
8. **Run tests before commit** - Use pre-commit hooks

## Troubleshooting

### Tests Fail Locally But Pass in CI

- Check Python version matches
- Check dependencies are installed
- Check ROS 2 environment is sourced

### Bench Tests Skip

- Check if on Jetson: `test -f /etc/nv_tegra_release`
- Set `FORCE_BENCH=1` to force run
- Check hardware is connected

### Integration Tests Fail

- Verify ROS 2 is installed and sourced
- Check workspace is built: `colcon build`
- Verify nodes are installed: `ros2 pkg list`

## See Also

- [Development Workflow](../development/WORKFLOW.md)
- [Node Management](../system/NODE_MANAGEMENT.md)
- [CI/CD](../deployment/GITHUB_SETUP.md)
