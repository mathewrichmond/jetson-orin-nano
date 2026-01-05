# Testing Quick Start

Quick reference for running tests in the Isaac robot system.

## Run Tests

```bash
# All tests (lint, unit, integration)
make test
# or
./scripts/testing/run_tests.sh all

# Specific level
make test-lint          # Lint tests
make test-unit          # Unit tests
make test-integration   # Integration tests
make test-bench         # Bench tests (hardware required)
```

## Bench Test: Camera Visualization

**On Jetson**:
```bash
./scripts/testing/bench_test_cameras.sh
```

**On MacBook**:
1. Open Foxglove Studio
2. Connect → ROS 2 → `ws://<jetson-ip>:9090`
3. Add Image panels for camera topics

## Pre-commit

```bash
# Install hooks
pre-commit install

# Run manually
pre-commit run --all-files
```

## CI

Tests run automatically on GitHub:
- Lint: Every push/PR
- Unit: Every push/PR
- Integration: Every push/PR (if ROS 2 available)
- Bench: Manual trigger only

See [Testing Guide](TESTING.md) for details.
