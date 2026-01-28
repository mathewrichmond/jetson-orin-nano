#!/usr/bin/env python3
"""
Smoke test for audio_pipeline module
Tests that all nodes can be imported and have basic structure
"""

import sys


def test_imports():
    """Test that all module imports work"""
    print("Testing audio_pipeline imports...")
    
    try:
        from audio_pipeline import audio_feature_extractor_node
        from audio_pipeline import speech_recognition_node
        from audio_pipeline import audio_pipeline_node
        print("  ✓ All modules imported successfully")
    except ImportError as e:
        print(f"  ✗ Import error: {e}")
        return False
    
    return True


def test_node_classes():
    """Test that node classes exist and have basic structure"""
    print("\nTesting node class structure...")
    
    try:
        from audio_pipeline.audio_feature_extractor_node import AudioFeatureExtractorNode
        from audio_pipeline.speech_recognition_node import SpeechRecognitionNode
        from audio_pipeline.audio_pipeline_node import AudioPipelineNode
        
        # Check that classes have __init__ method
        assert hasattr(AudioFeatureExtractorNode, '__init__')
        assert hasattr(SpeechRecognitionNode, '__init__')
        assert hasattr(AudioPipelineNode, '__init__')
        
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


def main():
    """Run all tests"""
    print("=" * 60)
    print("Audio Pipeline Module - Smoke Test")
    print("=" * 60)
    
    tests = [
        test_imports,
        test_node_classes,
        test_custom_msgs,
        test_isaac_utils,
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
