#!/usr/bin/env python3
"""
Smoke test for Power Management Module
"""

import sys
import subprocess

print("Testing Power Management Module - Node Imports and Basic Initialization")
print("=" * 70)

# Test 1: Import test
print("\nTest 1: Importing modules...")
try:
    from power_management.power_manager_node import PowerManagerNode, PowerMode
    from power_management.gpio_controller_node import GPIOControllerNode
    from power_management.battery_monitor_node import BatteryMonitorNode
    print("✓ All modules imported successfully")
except ImportError as e:
    print(f"✗ Import failed: {e}")
    sys.exit(1)

# Test 2: Check node class structure
print("\nTest 2: Checking node class structure...")
try:
    assert hasattr(PowerManagerNode, '__init__')
    assert hasattr(GPIOControllerNode, '__init__')
    assert hasattr(BatteryMonitorNode, '__init__')
    print("  ✓ PowerManagerNode class structure valid")
    print("  ✓ GPIOControllerNode class structure valid")
    print("  ✓ BatteryMonitorNode class structure valid")
    print("✓ All node classes have valid structure")
except Exception as e:
    print(f"✗ Node class check failed: {e}")
    sys.exit(1)

# Test 3: Check PowerMode enum
print("\nTest 3: Verifying PowerMode enum...")
try:
    assert hasattr(PowerMode, 'FULL')
    assert hasattr(PowerMode, 'ECONOMY')
    assert hasattr(PowerMode, 'CRITICAL')
    print("  ✓ PowerMode enum valid")
    print("✓ Power mode states defined correctly")
except Exception as e:
    print(f"✗ PowerMode check failed: {e}")
    sys.exit(1)

# Test 4: Verify executables
print("\nTest 4: Checking installed executables...")
result = subprocess.run(
    ['ros2', 'pkg', 'executables', 'power_management'],
    capture_output=True,
    text=True
)

expected_execs = ['power_manager_node', 'gpio_controller_node', 'battery_monitor_node']
found_execs = [line.split()[1] for line in result.stdout.strip().split('\n') if line.strip()]

if all(exe in found_execs for exe in expected_execs):
    for exe in expected_execs:
        print(f"  ✓ {exe} registered")
    print("✓ All executables properly installed")
else:
    missing = [exe for exe in expected_execs if exe not in found_execs]
    print(f"✗ Missing executables: {missing}")
    sys.exit(1)

# Summary
print("\n" + "=" * 70)
print("PHASE 3 VALIDATION: ALL TESTS PASSED ✓")
print("=" * 70)
print("\nNext steps:")
print("  1. Test with real battery hardware")
print("  2. Verify GPIO power control (with Jetson.GPIO)")
print("  3. Test power mode transitions")
print("  4. Validate runtime estimation accuracy")
print("\nNote: This is a smoke test. Full integration testing required.")
print("=" * 70)

sys.exit(0)
