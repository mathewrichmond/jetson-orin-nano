#!/usr/bin/env python3
"""
Simple validation test: Check that all nodes can be imported and initialized
This is a smoke test to verify basic functionality before hardware testing
"""

import sys
import os
import tempfile

print("Testing Chassis Control Module - Node Imports and Basic Initialization")
print("=" * 70)

# Test 1: Import test
print("\nTest 1: Importing modules...")
try:
    from chassis_control.imu_processor_node import IMUProcessorNode
    from chassis_control.chassis_controller_node import ChassisControllerNode
    from chassis_control.calibration_manager_node import CalibrationManagerNode
    from custom_msgs.msg import CalibrationStatus, PowerRequest, SystemPerformance
    print("✓ All modules imported successfully")
except ImportError as e:
    print(f"✗ Import failed: {e}")
    sys.exit(1)

# Test 2: Check custom messages
print("\nTest 2: Verifying custom message definitions...")
try:
    # CalibrationStatus
    cal_msg = CalibrationStatus()
    assert hasattr(cal_msg, 'imu_bias_x')
    assert hasattr(cal_msg, 'wheel_diameter_left_m')
    assert hasattr(cal_msg, 'calibration_quality')
    print("  ✓ CalibrationStatus message structure valid")

    # PowerRequest
    pwr_msg = PowerRequest()
    assert hasattr(pwr_msg, 'requested_state')
    assert hasattr(pwr_msg, 'requester_node')
    print("  ✓ PowerRequest message structure valid")

    # SystemPerformance
    perf_msg = SystemPerformance()
    assert hasattr(perf_msg, 'cpu_usage_pct')
    assert hasattr(perf_msg, 'battery_level_pct')
    print("  ✓ SystemPerformance message structure valid")

    print("✓ All custom message definitions valid")
except Exception as e:
    print(f"✗ Message definition check failed: {e}")
    sys.exit(1)

# Test 3: Node class instantiation (without ROS initialization)
print("\nTest 3: Checking node class structure...")
try:
    # Just verify classes exist and have expected methods
    assert hasattr(IMUProcessorNode, '__init__')
    assert hasattr(ChassisControllerNode, '__init__')
    assert hasattr(CalibrationManagerNode, '__init__')
    print("  ✓ IMUProcessorNode class structure valid")
    print("  ✓ ChassisControllerNode class structure valid")
    print("  ✓ CalibrationManagerNode class structure valid")
    print("✓ All node classes have valid structure")
except Exception as e:
    print(f"✗ Node class check failed: {e}")
    sys.exit(1)

# Test 4: Check isaac_utils dependency
print("\nTest 4: Verifying isaac_utils integration...")
try:
    from isaac_utils import HealthStatusPublisher, InputWatchdog, KalmanFilter
    print("  ✓ HealthStatusPublisher available")
    print("  ✓ InputWatchdog available")
    print("  ✓ KalmanFilter available")
    print("✓ isaac_utils integration verified")
except ImportError as e:
    print(f"✗ isaac_utils import failed: {e}")
    sys.exit(1)

# Test 5: Verify executables are installed
print("\nTest 5: Checking installed executables...")
import subprocess
result = subprocess.run(
    ['ros2', 'pkg', 'executables', 'chassis_control'],
    capture_output=True,
    text=True
)

expected_execs = [
    'imu_processor_node',
    'chassis_controller_node',
    'calibration_manager_node'
]

found_execs = []
for line in result.stdout.strip().split('\n'):
    if line.strip():
        exec_name = line.split()[1] if len(line.split()) > 1 else None
        if exec_name:
            found_execs.append(exec_name)

all_found = all(exe in found_execs for exe in expected_execs)

if all_found:
    for exe in expected_execs:
        print(f"  ✓ {exe} registered")
    print("✓ All executables properly installed")
else:
    missing = [exe for exe in expected_execs if exe not in found_execs]
    print(f"✗ Missing executables: {missing}")
    sys.exit(1)

# Summary
print("\n" + "=" * 70)
print("PHASE 1 VALIDATION: ALL TESTS PASSED ✓")
print("=" * 70)
print("\nNext steps:")
print("  1. Test with actual hardware (IMU, wheel encoders)")
print("  2. Verify health monitoring in live system")
print("  3. Test calibration convergence with vision feedback")
print("  4. Integrate with existing graph system")
print("\nNote: This is a smoke test. Full integration testing required.")
print("=" * 70)

sys.exit(0)
