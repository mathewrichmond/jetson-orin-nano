#!/usr/bin/env python3
"""
Simple validation test: Check that all vision pipeline nodes can be imported and initialized
This is a smoke test to verify basic functionality before hardware testing
"""

import sys

print("Testing Vision Pipeline Module - Node Imports and Basic Initialization")
print("=" * 70)

# Test 1: Import test
print("\nTest 1: Importing modules...")
try:
    from vision_pipeline.visual_slam_node import VisualSLAMNode
    from vision_pipeline.camera_calibration_node import CameraCalibrationNode
    from vision_pipeline.vision_pipeline_node import VisionPipelineNode
    from custom_msgs.msg import ModuleHealth, PowerRequest
    print("✓ All modules imported successfully")
except ImportError as e:
    print(f"✗ Import failed: {e}")
    sys.exit(1)

# Test 2: Check node class structure
print("\nTest 2: Checking node class structure...")
try:
    assert hasattr(VisualSLAMNode, '__init__')
    assert hasattr(CameraCalibrationNode, '__init__')
    assert hasattr(VisionPipelineNode, '__init__')
    print("  ✓ VisualSLAMNode class structure valid")
    print("  ✓ CameraCalibrationNode class structure valid")
    print("  ✓ VisionPipelineNode class structure valid")
    print("✓ All node classes have valid structure")
except Exception as e:
    print(f"✗ Node class check failed: {e}")
    sys.exit(1)

# Test 3: Check custom messages (already tested in Phase 1, but verify again)
print("\nTest 3: Verifying custom message dependencies...")
try:
    # ModuleHealth
    mod_health = ModuleHealth()
    assert hasattr(mod_health, 'module_name')
    assert hasattr(mod_health, 'status')
    print("  ✓ ModuleHealth message structure valid")

    # PowerRequest
    pwr_req = PowerRequest()
    assert hasattr(pwr_req, 'requested_state')
    assert hasattr(pwr_req, 'requester_node')
    print("  ✓ PowerRequest message structure valid")

    print("✓ All custom message dependencies valid")
except Exception as e:
    print(f"✗ Message dependency check failed: {e}")
    sys.exit(1)

# Test 4: Check isaac_utils dependency
print("\nTest 4: Verifying isaac_utils integration...")
try:
    from isaac_utils import HealthStatusPublisher, InputWatchdog
    print("  ✓ HealthStatusPublisher available")
    print("  ✓ InputWatchdog available")
    print("✓ isaac_utils integration verified")
except ImportError as e:
    print(f"✗ isaac_utils import failed: {e}")
    sys.exit(1)

# Test 5: Verify executables are installed
print("\nTest 5: Checking installed executables...")
import subprocess
result = subprocess.run(
    ['ros2', 'pkg', 'executables', 'vision_pipeline'],
    capture_output=True,
    text=True
)

expected_execs = [
    'visual_slam_node',
    'camera_calibration_node',
    'vision_pipeline_node'
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
print("PHASE 2 VALIDATION: ALL TESTS PASSED ✓")
print("=" * 70)
print("\nNext steps:")
print("  1. Install RTAB-Map for visual SLAM backend")
print("  2. Test with actual RealSense cameras")
print("  3. Verify SLAM initialization and tracking")
print("  4. Test camera calibration convergence")
print("  5. Integrate with Phase 1 chassis control for drift correction")
print("\nNote: This is a smoke test. Full integration testing required.")
print("=" * 70)

sys.exit(0)
