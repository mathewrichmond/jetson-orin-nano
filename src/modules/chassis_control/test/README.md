# Chassis Control Module Tests

Individual test scripts for validating chassis control nodes.

## Test Files

### 1. test_imu_processor.py
Tests the IMU Processor Node:
- IMU data reception and publishing
- Kalman filter functionality
- Message rate verification
- Acceleration filtering

**Run:**
```bash
source install/setup.bash
python3 src/modules/chassis_control/test/test_imu_processor.py
```

### 2. test_calibration_manager.py
Tests the Calibration Manager Node:
- Drift computation from vision and odometry
- Calibration parameter updates
- File save/load functionality
- Status message publishing

**Run:**
```bash
source install/setup.bash
python3 src/modules/chassis_control/test/test_calibration_manager.py
```

### 3. test_chassis_controller.py
Tests the Chassis Controller Node:
- Odometry computation
- Pose estimation
- Command velocity processing
- Vision feedback integration

**Run:**
```bash
source install/setup.bash
python3 src/modules/chassis_control/test/test_chassis_controller.py
```

### 4. mock_imu_publisher.py
Utility to publish synthetic IMU data for manual testing.

**Run:**
```bash
source install/setup.bash
python3 src/modules/chassis_control/test/mock_imu_publisher.py
```

## Expected Test Results

All tests should:
1. Print "PASS" at the end
2. Exit with code 0
3. Show message reception at expected rates (40-50 Hz)
4. Display meaningful data values

## Current Limitations

- Tests run in isolation without real hardware
- Mock data is used instead of actual sensors
- Some tests may show warnings without encoder/camera data
- Tests are not integrated into a formal testing framework yet

## Future Work

### Comprehensive Testing Framework Needed

**TODO:** Establish a proper testing framework that includes:

1. **Unit Tests**
   - pytest-based unit tests for individual functions
   - Mock-based testing for ROS 2 dependencies
   - Coverage reports

2. **Integration Tests**
   - Multi-node integration tests
   - Hardware-in-the-loop testing
   - End-to-end scenario tests

3. **Continuous Integration**
   - Automated test runs on commits
   - Build verification
   - Regression testing

4. **Test Organization**
   - Separate unit tests from integration tests
   - Standard directory structure (`test/unit/`, `test/integration/`)
   - Shared test fixtures and utilities

5. **Documentation**
   - Test writing guidelines
   - CI/CD setup documentation
   - Coverage goals and policies

### Suggested Framework Stack

```yaml
Testing Tools:
  - pytest: Primary test runner
  - pytest-ros: ROS 2 integration for pytest
  - pytest-cov: Coverage reporting
  - pytest-mock: Mocking utilities
  - colcon test: ROS 2 native testing

CI/CD:
  - GitHub Actions or Jenkins
  - Automated colcon build + test
  - Coverage badges
```

### Example Future Structure

```
src/modules/chassis_control/
├── chassis_control/          # Source code
├── test/
│   ├── unit/                 # Unit tests (fast, no ROS dependencies)
│   │   ├── test_kalman_filter.py
│   │   ├── test_odometry_math.py
│   │   └── test_calibration_logic.py
│   ├── integration/          # Integration tests (with ROS nodes)
│   │   ├── test_imu_processor.py
│   │   ├── test_calibration_manager.py
│   │   └── test_chassis_controller.py
│   ├── fixtures/             # Shared test fixtures
│   │   ├── mock_nodes.py
│   │   └── test_data.py
│   └── conftest.py           # pytest configuration
├── CMakeLists.txt
└── package.xml
```

## Notes

- Current tests are **manual validation scripts**, not formal unit tests
- Tests are **functional** but not **comprehensive**
- Hardware integration testing still required with real robot
- Consider these as **smoke tests** until proper framework established

## Running All Tests

```bash
#!/bin/bash
# run_all_tests.sh

source install/setup.bash

echo "Running IMU Processor Test..."
python3 src/modules/chassis_control/test/test_imu_processor.py
IMU_RESULT=$?

echo "Running Calibration Manager Test..."
python3 src/modules/chassis_control/test/test_calibration_manager.py
CALIB_RESULT=$?

echo "Running Chassis Controller Test..."
python3 src/modules/chassis_control/test/test_chassis_controller.py
CHASSIS_RESULT=$?

echo ""
echo "================================"
echo "Test Results Summary"
echo "================================"
echo "IMU Processor: $([ $IMU_RESULT -eq 0 ] && echo 'PASS' || echo 'FAIL')"
echo "Calibration Manager: $([ $CALIB_RESULT -eq 0 ] && echo 'PASS' || echo 'FAIL')"
echo "Chassis Controller: $([ $CHASSIS_RESULT -eq 0 ] && echo 'PASS' || echo 'FAIL')"
echo "================================"

exit $(( $IMU_RESULT + $CALIB_RESULT + $CHASSIS_RESULT ))
```
