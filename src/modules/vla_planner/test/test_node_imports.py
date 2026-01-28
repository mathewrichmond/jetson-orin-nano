#!/usr/bin/env python3
"""
Smoke test for vla_planner module
Tests that all nodes can be imported and have basic structure
"""

import sys


def test_imports():
    """Test that all module imports work"""
    print("Testing vla_planner imports...")
    
    try:
        from vla_planner import vla_controller_node
        from vla_planner import action_executor_node
        from vla_planner import planner_node
        print("  ✓ All modules imported successfully")
    except ImportError as e:
        print(f"  ✗ Import error: {e}")
        return False
    
    return True


def test_node_classes():
    """Test that node classes exist and have basic structure"""
    print("\nTesting node class structure...")
    
    try:
        from vla_planner.vla_controller_node import VLAControllerNode
        from vla_planner.action_executor_node import ActionExecutorNode
        from vla_planner.planner_node import PlannerNode
        
        # Check that classes have __init__ method
        assert hasattr(VLAControllerNode, '__init__')
        assert hasattr(ActionExecutorNode, '__init__')
        assert hasattr(PlannerNode, '__init__')
        
        print("  ✓ All node classes have valid structure")
    except (ImportError, AssertionError) as e:
        print(f"  ✗ Structure error: {e}")
        return False
    
    return True


def test_custom_msgs():
    """Test that custom message dependencies are available"""
    print("\nTesting custom message imports...")
    
    try:
        from custom_msgs.msg import ModuleHealth, NodeHealth, PowerRequest
        print("  ✓ Custom messages imported successfully")
    except ImportError as e:
        print(f"  ✗ Custom message import error: {e}")
        return False
    
    return True


def test_isaac_utils():
    """Test that isaac_utils dependencies are available"""
    print("\nTesting isaac_utils imports...")
    
    try:
        from isaac_utils import HealthStatusPublisher, InputWatchdog
        print("  ✓ isaac_utils imported successfully")
    except ImportError as e:
        print(f"  ✗ isaac_utils import error: {e}")
        return False
    
    return True


def test_enums():
    """Test that enum classes are defined"""
    print("\nTesting enum definitions...")
    
    try:
        from vla_planner.action_executor_node import ExecutionState
        from vla_planner.planner_node import TaskState
        
        # Check enum values
        assert hasattr(ExecutionState, 'IDLE')
        assert hasattr(ExecutionState, 'EXECUTING')
        assert hasattr(TaskState, 'PENDING')
        assert hasattr(TaskState, 'ACTIVE')
        
        print("  ✓ All enums defined correctly")
    except (ImportError, AssertionError) as e:
        print(f"  ✗ Enum error: {e}")
        return False
    
    return True


def main():
    """Run all tests"""
    print("=" * 60)
    print("VLA Planner Module - Smoke Test")
    print("=" * 60)
    
    tests = [
        test_imports,
        test_node_classes,
        test_custom_msgs,
        test_isaac_utils,
        test_enums,
    ]
    
    results = []
    for test in tests:
        results.append(test())
    
    print("\n" + "=" * 60)
    if all(results):
        print("✅ ALL TESTS PASSED")
        print("=" * 60)
        return 0
    else:
        print("❌ SOME TESTS FAILED")
        print("=" * 60)
        return 1


if __name__ == "__main__":
    sys.exit(main())
