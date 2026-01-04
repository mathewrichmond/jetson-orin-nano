#!/usr/bin/env python3
"""Quick test script for hello_world"""
import sys
sys.path.insert(0, '/home/nano/ros2_ws/install/hello_world/lib/python3.10/site-packages')

try:
    from hello_world.hello_world_node import HelloWorldNode
    print("✅ Hello World Node imported successfully!")
    print("✅ Development install is working!")
    print("")
    print("To run the node:")
    print("  source /opt/ros/humble/setup.bash")
    print("  source ~/ros2_ws/install/setup.bash")
    print("  python3 ~/ros2_ws/install/hello_world/lib/python3.10/site-packages/hello_world/hello_world_node.py")
    sys.exit(0)
except Exception as e:
    print(f"❌ Error: {e}")
    sys.exit(1)
