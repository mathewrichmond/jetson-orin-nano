#!/bin/bash
# Run all chassis_control module tests

set -e  # Exit on error

# Change to workspace root
cd "$(dirname "$0")"/../../../..

# Source ROS 2 environment
source install/setup.bash

echo "================================"
echo "Chassis Control Module Tests"
echo "================================"
echo ""

# Track results
TESTS_PASSED=0
TESTS_FAILED=0

# Test 1: IMU Processor
echo "Test 1/3: IMU Processor Node"
echo "----------------------------"
if python3 src/modules/chassis_control/test/test_imu_processor.py; then
    echo "✓ IMU Processor: PASS"
    ((TESTS_PASSED++))
else
    echo "✗ IMU Processor: FAIL"
    ((TESTS_FAILED++))
fi
echo ""

# Test 2: Calibration Manager
echo "Test 2/3: Calibration Manager Node"
echo "-----------------------------------"
if python3 src/modules/chassis_control/test/test_calibration_manager.py; then
    echo "✓ Calibration Manager: PASS"
    ((TESTS_PASSED++))
else
    echo "✗ Calibration Manager: FAIL"
    ((TESTS_FAILED++))
fi
echo ""

# Test 3: Chassis Controller
echo "Test 3/3: Chassis Controller Node"
echo "----------------------------------"
if python3 src/modules/chassis_control/test/test_chassis_controller.py; then
    echo "✓ Chassis Controller: PASS"
    ((TESTS_PASSED++))
else
    echo "✗ Chassis Controller: FAIL"
    ((TESTS_FAILED++))
fi
echo ""

# Summary
echo "================================"
echo "Test Results Summary"
echo "================================"
echo "Tests Passed: $TESTS_PASSED/3"
echo "Tests Failed: $TESTS_FAILED/3"
echo "================================"
echo ""

if [ $TESTS_FAILED -eq 0 ]; then
    echo "All tests PASSED ✓"
    exit 0
else
    echo "Some tests FAILED ✗"
    exit 1
fi
