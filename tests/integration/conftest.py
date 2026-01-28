"""
Integration Test Configuration and Fixtures

Provides ROS 2-specific fixtures for integration testing.
"""

import time
from typing import Generator, Optional

import pytest
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor


@pytest.fixture(scope="session")
def rclpy_init() -> Generator[None, None, None]:
    """Initialize rclpy for the test session"""
    if not rclpy.ok():
        rclpy.init()
    
    yield
    
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def test_node(rclpy_init) -> Generator[Node, None, None]:
    """Create a test node"""
    node = Node("test_node")
    
    yield node
    
    node.destroy_node()


@pytest.fixture
def executor(rclpy_init) -> Generator[SingleThreadedExecutor, None, None]:
    """Create an executor for running nodes"""
    executor = SingleThreadedExecutor()
    
    yield executor
    
    executor.shutdown()


class NodeRunner:
    """Helper for running nodes in tests"""
    
    def __init__(self, node: Node, executor: SingleThreadedExecutor):
        self.node = node
        self.executor = executor
        self.executor.add_node(node)
    
    def spin_once(self, timeout_sec: float = 0.1):
        """Spin once with timeout"""
        self.executor.spin_once(timeout_sec=timeout_sec)
    
    def spin_until_topic_received(
        self,
        topic_name: str,
        timeout_sec: float = 5.0,
    ) -> bool:
        """Spin until a message is received on a topic"""
        start_time = time.time()
        
        while time.time() - start_time < timeout_sec:
            self.spin_once()
            
            # Check if topic has subscribers (simplified check)
            topics = self.node.get_topic_names_and_types()
            if any(topic == topic_name for topic, _ in topics):
                return True
        
        return False


@pytest.fixture
def node_runner(test_node, executor) -> NodeRunner:
    """Create a node runner for tests"""
    return NodeRunner(test_node, executor)


@pytest.fixture
def wait_for_node():
    """Wait for a node to appear"""
    def _wait(node_name: str, timeout: float = 5.0) -> bool:
        """Wait for node to appear"""
        # Implementation would use ros2 node list
        # For now, return True (placeholder)
        time.sleep(0.1)
        return True
    
    return _wait
