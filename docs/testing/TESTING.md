## Testing Framework - Isaac Robot System

**Status**: Phase 6 ✅ | Comprehensive testing and CI/CD

---

## Overview

The Isaac robot system includes a comprehensive testing framework with:

- **Unit Tests** - Hermetic, fast tests of individual functions/classes
- **Integration Tests** - Test graphs with spoofed data or log replay
- **Mock Hardware** - Test without real devices
- **Log Replay** - Replay recorded ROS 2 bags for reproducible testing
- **CI/CD** - Automated testing and deployment via GitHub Actions

---

## Quick Start

### Run All Tests

```bash
# Run locally
./scripts/testing/run_tests.sh all

# With coverage
COVERAGE=true ./scripts/testing/run_tests.sh all

# Verbose mode
VERBOSE=true ./scripts/testing/run_tests.sh all
```

### Run Specific Test Suites

```bash
# Unit tests only (fast, no ROS 2 required)
./scripts/testing/run_tests.sh unit

# Integration tests (requires ROS 2)
./scripts/testing/run_tests.sh integration

# Using pytest directly
pytest tests/unit/ -v -m "unit"
pytest tests/integration/ -v -m "integration"
```

---

## Test Organization

```
tests/
├── conftest.py              # Shared pytest fixtures
├── unit/                    # Unit tests (hermetic)
│   └── test_*.py
├── integration/             # Integration tests (ROS 2)
│   ├── conftest.py          # ROS 2 fixtures
│   └── test_*.py
├── fixtures/                # Test data and utilities
│   ├── log_replay.py        # Bag replay system
│   └── sample_bags/         # Sample bag files
└── mocks/                   # Mock implementations
    ├── mock_sensor_data.py  # Mock data generators
    └── mock_hardware_nodes.py # Mock ROS 2 nodes
```

---

## Unit Testing

### Philosophy

Unit tests should be:
- **Hermetic** - No external dependencies (no ROS 2, no hardware, no network)
- **Fast** - Complete in < 100ms each
- **Isolated** - No side effects between tests
- **Deterministic** - Same result every time

### Example Unit Test

```python
import pytest

@pytest.mark.unit
def test_example():
    """Test a pure function"""
    result = my_function(5)
    assert result == 10

@pytest.mark.unit
class TestMyClass:
    """Test a class"""
    
    def test_initialization(self):
        obj = MyClass(param=5)
        assert obj.param == 5
    
    def test_method(self):
        obj = MyClass(param=5)
        result = obj.calculate()
        assert result == 25

@pytest.mark.unit
@pytest.mark.parametrize("input,expected", [
    (0, 0),
    (1, 2),
    (5, 10),
])
def test_parametrized(input, expected):
    """Parametrized test"""
    assert input * 2 == expected
```

### Running Unit Tests

```bash
# All unit tests
pytest tests/unit/ -m "unit"

# Specific file
pytest tests/unit/test_example.py

# Specific test
pytest tests/unit/test_example.py::test_example

# With coverage
pytest tests/unit/ --cov=src --cov-report=html

# Verbose
pytest tests/unit/ -v -s
```

---

## Integration Testing

### Philosophy

Integration tests:
- **Test ROS 2 graphs** - Launch nodes and verify behavior
- **Use mock hardware** - No real devices required
- **Spoof data** - Generate realistic sensor data
- **Replay logs** - Use recorded bags for reproducibility

### Test Graphs

Integration tests launch minimal graphs with mock hardware:

```python
import pytest
import subprocess
import time

@pytest.mark.integration
class TestMinimalGraph:
    """Test minimal graph deployment"""
    
    @pytest.fixture
    def launch_graph(self, ros_domain_id):
        """Launch test graph"""
        env = {"ROS_DOMAIN_ID": str(ros_domain_id), "MOCK_HARDWARE": "true"}
        
        process = subprocess.Popen(
            ["ros2", "launch", "isaac_robot", "graph.launch.py",
             "graph:=minimal_graph.yaml"],
            env=env
        )
        
        time.sleep(5)  # Wait for startup
        yield process
        
        process.terminate()
        process.wait()
    
    def test_nodes_present(self, launch_graph, ros_domain_id):
        """Verify expected nodes are running"""
        result = subprocess.run(
            ["ros2", "node", "list"],
            env={"ROS_DOMAIN_ID": str(ros_domain_id)},
            capture_output=True,
            text=True
        )
        
        assert "/system_monitor" in result.stdout
        assert "/health_monitor" in result.stdout
```

### Spoofing Data

Use mock data generators for testing:

```python
from tests.mocks import MockIMUData, MockCameraData

# Generate mock IMU data
imu = MockIMUData(frequency=50.0, noise_level=0.01)
data = imu.generate()

# Generate mock camera frame
camera = MockCameraData(width=640, height=480)
frame = camera.generate()
```

### Running Integration Tests

```bash
# All integration tests
pytest tests/integration/ -m "integration"

# Exclude slow tests
pytest tests/integration/ -m "integration and not slow"

# Exclude hardware tests
pytest tests/integration/ -m "integration and not hardware"

# Specific graph test
pytest tests/integration/test_graph_minimal.py -v
```

---

## Log Replay Testing

### Recording Logs

Record ROS 2 bags for later replay:

```bash
# Record all topics for 30 seconds
ros2 bag record --all --duration 30 -o test_scenario_1

# Record specific topics
ros2 bag record /camera/image /imu/data -o imu_camera_test
```

Or use the Python API:

```python
from tests.fixtures import BagRecorder

recorder = BagRecorder(
    output_path=Path("test_data.bag"),
    topics=["/camera/image", "/imu/data"]
)

recorder.start_recording(max_duration=30.0)
# ... run test scenario ...
recorder.stop_recording()
```

### Replaying Logs

Replay bags in integration tests:

```python
from tests.fixtures import LogReplayer

@pytest.mark.integration
def test_with_bag_replay():
    """Test using recorded bag file"""
    replayer = LogReplayer(
        bag_path=Path("tests/fixtures/sample_bags/test_scenario_1"),
        domain_id=42
    )
    
    # Start replay
    replayer.start_replay(rate=1.0, loop=False)
    
    # Wait for replay to start
    time.sleep(2)
    
    # Run your test assertions
    # ... check that nodes process the data correctly ...
    
    # Stop replay
    replayer.stop_replay()
```

### Sample Bags

Store sample bags in `tests/fixtures/sample_bags/`:
- `minimal_system/` - System monitor + health monitor
- `chassis_control/` - IMU + motor commands
- `vision_pipeline/` - Camera + depth images
- `full_system/` - Complete robot operation

---

## Mock Hardware

### Mock Nodes

Use mock hardware nodes for testing without devices:

```python
from tests.mocks import MockIMUNode, MockCameraNode

# Launch mock IMU
mock_imu = MockIMUNode(node_name="mock_imu")

# Launch mock camera
mock_camera = MockCameraNode(node_name="mock_camera")
```

Or launch via command line:

```bash
# Mock IMU
ros2 run isaac_utils mock_imu_node --ros-args -p publish_rate:=50.0

# Mock camera
ros2 run isaac_utils mock_camera_node --ros-args -p fps:=30.0
```

### Mock Data Generators

Available mock data generators:

```python
from tests.mocks import (
    MockIMUData,
    MockCameraData,
    MockAudioData,
    MockBatteryData,
    MockOdometryData,
)

# IMU data
imu = MockIMUData(frequency=50.0, noise_level=0.01)
data = imu.generate()  # Returns dict with accelerometer + gyro

# Camera frames
camera = MockCameraData(width=640, height=480, fps=30.0)
frame = camera.generate()  # Returns numpy array (H, W, 3)

# Audio samples
audio = MockAudioData(sample_rate=16000, channels=1)
samples = audio.generate(duration_sec=0.1)  # Returns numpy array

# Battery readings
battery = MockBatteryData(initial_voltage=12.6)
reading = battery.generate()  # Returns dict with voltage, current, temp

# Odometry
odom = MockOdometryData()
pose = odom.generate(linear_vel=0.5, angular_vel=0.1)
```

---

## Test Markers

### Available Markers

| Marker | Description | Use Case |
|--------|-------------|----------|
| `@pytest.mark.unit` | Unit test | Fast, hermetic tests |
| `@pytest.mark.integration` | Integration test | ROS 2 graph tests |
| `@pytest.mark.hardware` | Hardware test | Requires real devices |
| `@pytest.mark.slow` | Slow test (> 1s) | Long-running tests |
| `@pytest.mark.gpu` | GPU test | Requires CUDA |
| `@pytest.mark.smoke` | Smoke test | Quick sanity check |

### Running Specific Markers

```bash
# Only unit tests
pytest -m "unit"

# Only integration tests
pytest -m "integration"

# Integration tests, but not slow
pytest -m "integration and not slow"

# No hardware tests (default)
pytest -m "not hardware"

# Include hardware tests
pytest -m "hardware" --hardware

# GPU tests only
pytest -m "gpu" --gpu
```

---

## Continuous Integration (CI)

### GitHub Actions Workflows

#### Test Workflow (`.github/workflows/test.yml`)

Runs on every push and PR:
1. **Unit Tests** - Fast hermetic tests
2. **Integration Tests** - ROS 2 graph tests with mock hardware
3. **Linting** - black, isort, flake8, pylint
4. **Build Test** - Verify all packages build

**Triggers**:
- Push to `main` or `dev`
- Pull requests to `main`
- Manual dispatch

**Jobs**:
- `unit-tests` - Python unit tests with coverage
- `integration-tests` - ROS 2 integration tests
- `lint` - Code quality checks
- `build-test` - Full build verification

#### Deployment Workflow (`.github/workflows/deploy.yml`)

Deploys to robot hosts:
1. **Deploy to Jetson** - Rsync + build
2. **Deploy to Pi** - Rsync + build (if dual-compute)
3. **Health Check** - Verify deployment
4. **Create Release** - For tagged versions

**Triggers**:
- Push to `main`
- Tags matching `v*`
- Manual dispatch (select hosts)

**Required Secrets**:
- `JETSON_SSH_KEY` - SSH private key for Jetson
- `JETSON_HOST` - Jetson hostname/IP
- `PI_SSH_KEY` - SSH private key for Pi
- `PI_HOST` - Pi hostname/IP

### Setting Up Secrets

In GitHub: **Settings** → **Secrets and variables** → **Actions** → **New repository secret**

```bash
# Generate SSH key for deployment
ssh-keygen -t ed25519 -f ~/.ssh/isaac_deploy_key -N ""

# Add public key to robot
ssh-copy-id -i ~/.ssh/isaac_deploy_key.pub nano@isaac-jetson.local

# Add private key to GitHub secrets
# Name: JETSON_SSH_KEY
# Value: Contents of ~/.ssh/isaac_deploy_key
```

### Manual Deployment Trigger

**GitHub** → **Actions** → **Deploy to Robot** → **Run workflow**

Select:
- Environment: dev, staging, or production
- Hosts: jetson, pi, or both

---

## Remote Deployment

### Deploy from Development Machine

```bash
# Deploy to Jetson
./scripts/deployment/remote_deploy.sh jetson

# Deploy to Pi
./scripts/deployment/remote_deploy.sh pi

# Deploy to all hosts
./scripts/deployment/remote_deploy.sh all

# Dry run (show what would be deployed)
DRY_RUN=true ./scripts/deployment/remote_deploy.sh jetson

# Deploy without building
BUILD_AFTER_DEPLOY=false ./scripts/deployment/remote_deploy.sh jetson

# Deploy and restart services
RESTART_SERVICES=true ./scripts/deployment/remote_deploy.sh jetson
```

### Environment Variables

```bash
# Jetson configuration
export JETSON_USER=nano
export JETSON_HOST=isaac-jetson.local

# Pi configuration
export PI_USER=pi
export PI_HOST=isaac-pi.local

# Deployment options
export DRY_RUN=false
export BUILD_AFTER_DEPLOY=true
export RESTART_SERVICES=false
export DEPLOYMENT_CONFIG=dual_compute
```

### Multi-Target Deployment Strategy

**Scenario 1: Local Development on Jetson**
```bash
# Currently: Developing directly on Jetson
# No deployment needed, just build locally
colcon build --symlink-install
```

**Scenario 2: Development Machine → Jetson**
```bash
# From dev machine
./scripts/deployment/remote_deploy.sh jetson

# Or manually
rsync -avz --exclude '.git' --exclude 'build' \
    ./ nano@isaac-jetson.local:~/src/jetson-orin-nano/

ssh nano@isaac-jetson.local "cd ~/src/jetson-orin-nano && colcon build --symlink-install"
```

**Scenario 3: Development Machine → Pi + Jetson** (Future)
```bash
# Deploy to both hosts
./scripts/deployment/remote_deploy.sh all

# Or deploy sequentially
./scripts/deployment/remote_deploy.sh jetson
./scripts/deployment/remote_deploy.sh pi

# Restart services on both
RESTART_SERVICES=true ./scripts/deployment/remote_deploy.sh all
```

**Scenario 4: CI/CD → Production**
```bash
# Automatic via GitHub Actions on push to main
# Or manual trigger via GitHub UI

# Manual deployment with specific config
DEPLOYMENT_CONFIG=production ./scripts/deployment/remote_deploy.sh all
```

---

## Test Development Workflow

### 1. Write Unit Tests First (TDD)

```python
# tests/unit/test_new_feature.py
import pytest

@pytest.mark.unit
def test_new_feature():
    """Test new feature implementation"""
    # Write test first (it will fail)
    result = new_feature(input_data)
    assert result == expected_output

# Now implement new_feature() to make test pass
```

### 2. Add Integration Tests

```python
# tests/integration/test_new_feature_integration.py
import pytest

@pytest.mark.integration
def test_new_feature_in_graph(launch_graph):
    """Test new feature in ROS 2 graph"""
    # Launch graph with new feature
    # Verify it integrates correctly
    pass
```

### 3. Run Tests Locally

```bash
# Unit tests (fast iteration)
pytest tests/unit/test_new_feature.py -v

# Integration tests
pytest tests/integration/test_new_feature_integration.py -v

# All tests
./scripts/testing/run_tests.sh all
```

### 4. Push and Let CI Run

```bash
git add tests/
git commit -m "test: Add tests for new feature"
git push

# GitHub Actions will automatically:
# 1. Run all unit tests
# 2. Run integration tests
# 3. Check code quality
# 4. Verify build
```

### 5. Review Coverage

```bash
# Generate local coverage report
COVERAGE=true ./scripts/testing/run_tests.sh all

# Open HTML report
open htmlcov/index.html

# Or view on GitHub (uploaded to Codecov)
```

---

## Best Practices

### Unit Tests

✅ **DO**:
- Test pure functions without side effects
- Use fixtures for common setup
- Parametrize tests for multiple inputs
- Keep tests fast (< 100ms)
- Mock external dependencies

❌ **DON'T**:
- Access real hardware
- Use ROS 2 (that's integration testing)
- Make network calls
- Write to filesystem (use tmp_path fixture)
- Depend on test execution order

### Integration Tests

✅ **DO**:
- Test complete ROS 2 graphs
- Use mock hardware by default
- Use isolated ROS domain (42)
- Clean up launched processes
- Use fixtures for graph lifecycle

❌ **DON'T**:
- Test on production domain (0)
- Leave processes running
- Assume hardware is available
- Make tests depend on each other

### Mock Hardware

✅ **DO**:
- Generate realistic data (with noise)
- Match real hardware characteristics
- Document mock behavior
- Make mocks configurable

❌ **DON'T**:
- Generate perfect/deterministic data
- Ignore timing constraints
- Skip validation

---

## Troubleshooting

### Tests Won't Run

**Issue**: `ImportError: No module named 'pytest'`

**Fix**:
```bash
pip install pytest pytest-cov pytest-mock pytest-timeout
```

**Issue**: `ImportError: No module named 'rclpy'`

**Fix**:
```bash
source /opt/ros/humble/setup.bash
```

**Issue**: `No nodes found`

**Fix**:
```bash
# Check ROS_DOMAIN_ID
echo $ROS_DOMAIN_ID  # Should be 42 for tests

# Ensure mock hardware is enabled
export MOCK_HARDWARE=true
```

### Integration Tests Fail

**Issue**: Timeout waiting for nodes

**Fix**:
- Increase timeout in test
- Check if graph launched successfully
- Verify ROS 2 domain is correct

**Issue**: Nodes from previous tests still running

**Fix**:
```bash
# Kill all ROS processes
pkill -f ros2

# Stop daemon
ros2 daemon stop
```

### CI Failing

**Issue**: Tests pass locally but fail in CI

**Fix**:
- Check GitHub Actions logs
- Ensure all dependencies are in `requirements-dev.txt`
- Verify ROS 2 packages are available in CI
- Check for race conditions (timing issues)

### Deployment Fails

**Issue**: SSH connection fails

**Fix**:
```bash
# Test SSH manually
ssh nano@isaac-jetson.local

# Check SSH keys
ls ~/.ssh/

# Verify known_hosts
ssh-keyscan isaac-jetson.local >> ~/.ssh/known_hosts
```

**Issue**: Build fails on remote

**Fix**:
- Check remote ROS 2 installation
- Verify dependencies are installed
- Check disk space on robot

---

## Performance Targets

### Unit Tests
- **Total runtime**: < 30 seconds
- **Individual test**: < 100ms
- **Coverage**: > 80%

### Integration Tests
- **Total runtime**: < 5 minutes
- **Individual test**: < 30 seconds
- **Flakiness**: < 1%

### CI Pipeline
- **Total pipeline**: < 15 minutes
- **Unit tests job**: < 2 minutes
- **Integration tests job**: < 10 minutes
- **Build test job**: < 5 minutes

---

## Future Enhancements

- [ ] Hardware-in-the-loop (HIL) testing
- [ ] Performance benchmarking tests
- [ ] Stress testing framework
- [ ] Chaos engineering for failure modes
- [ ] Visual regression testing
- [ ] Automated test data generation
- [ ] Test result dashboards

---

## See Also

- **GitHub Actions Docs**: `.github/workflows/`
- **Pytest Documentation**: https://docs.pytest.org/
- **Mock Hardware**: `tests/mocks/`
- **Test Fixtures**: `tests/fixtures/`
- **Deployment Guide**: `docs/deployment/DEPLOYMENT.md`

---

**Last Updated**: 2026-01-27  
**Status**: Phase 6 Complete  
**Next**: Hardware validation and performance benchmarking
