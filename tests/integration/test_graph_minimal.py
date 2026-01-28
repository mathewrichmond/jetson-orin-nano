"""
Integration Test: Minimal Graph

Tests the minimal graph configuration with spoofed data.
"""

import pytest
import subprocess
import time
import signal
import os
from pathlib import Path


@pytest.mark.integration
@pytest.mark.slow
class TestMinimalGraph:
    """Test minimal graph deployment"""
    
    @pytest.fixture
    def launch_minimal_graph(self, ros_domain_id, repo_root):
        """Launch minimal graph for testing"""
        # Set environment
        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = str(ros_domain_id)
        env["MOCK_HARDWARE"] = "true"
        
        # Launch process
        launch_cmd = [
            "ros2", "launch",
            "isaac_robot", "graph.launch.py",
            f"graph:=minimal_graph.yaml",
            f"group:=all",
        ]
        
        process = subprocess.Popen(
            launch_cmd,
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            cwd=repo_root,
        )
        
        # Wait for launch to complete
        time.sleep(5)
        
        yield process
        
        # Cleanup
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
    
    def test_minimal_graph_launches(self, launch_minimal_graph):
        """Test that minimal graph launches successfully"""
        process = launch_minimal_graph
        
        # Check process is running
        assert process.poll() is None, "Process terminated unexpectedly"
        
        # Wait a bit for nodes to initialize
        time.sleep(2)
        
        # Process should still be running
        assert process.poll() is None, "Process crashed"
    
    def test_minimal_graph_nodes_present(self, launch_minimal_graph, ros_domain_id):
        """Test that expected nodes are present"""
        process = launch_minimal_graph
        
        # Wait for nodes to start
        time.sleep(3)
        
        # Get node list
        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = str(ros_domain_id)
        
        result = subprocess.run(
            ["ros2", "node", "list"],
            env=env,
            capture_output=True,
            text=True,
        )
        
        node_list = result.stdout
        
        # Check for expected nodes (minimal graph)
        expected_nodes = [
            "/system_monitor",
            "/health_monitor",
        ]
        
        for node_name in expected_nodes:
            assert node_name in node_list, f"Node {node_name} not found"
    
    @pytest.mark.slow
    def test_minimal_graph_topics_published(self, launch_minimal_graph, ros_domain_id):
        """Test that expected topics are being published"""
        process = launch_minimal_graph
        
        # Wait for nodes to start publishing
        time.sleep(5)
        
        # Check topic list
        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = str(ros_domain_id)
        
        result = subprocess.run(
            ["ros2", "topic", "list"],
            env=env,
            capture_output=True,
            text=True,
        )
        
        topic_list = result.stdout
        
        # Check for expected topics
        expected_topics = [
            "/system/performance",
            "/system/health",
        ]
        
        for topic_name in expected_topics:
            assert topic_name in topic_list, f"Topic {topic_name} not found"
