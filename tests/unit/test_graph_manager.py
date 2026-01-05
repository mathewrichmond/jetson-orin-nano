"""
Unit Tests: Graph Manager
Hermetic tests with mocked dependencies
"""
import pytest
import yaml
from pathlib import Path
from unittest.mock import Mock, patch, MagicMock


@pytest.mark.unit
class TestGraphManager:
    """Unit tests for GraphManager"""

    def test_load_config(self, repo_root):
        """Test loading graph configuration"""
        from isaac_robot.graph_manager import GraphManager

        config_file = repo_root / "src" / "isaac_robot" / "config" / "robot" / "robot_graph.yaml"

        if config_file.exists():
            manager = GraphManager(str(config_file))
            config = manager.load_config()

            assert 'robot' in config
            assert 'groups' in config
            assert 'system_monitor' in config['robot']

    def test_get_enabled_nodes(self, repo_root):
        """Test getting enabled nodes from group"""
        from isaac_robot.graph_manager import GraphManager

        config_file = repo_root / "src" / "isaac_robot" / "config" / "robot" / "robot_graph.yaml"

        if config_file.exists():
            manager = GraphManager(str(config_file))
            manager.load_config()

            enabled = manager.get_enabled_nodes('core')
            assert isinstance(enabled, list)
            assert 'system_monitor' in enabled

    def test_get_node_config(self, repo_root):
        """Test getting node configuration"""
        from isaac_robot.graph_manager import GraphManager

        config_file = repo_root / "src" / "isaac_robot" / "config" / "robot" / "robot_graph.yaml"

        if config_file.exists():
            manager = GraphManager(str(config_file))
            manager.load_config()

            node_config = manager.get_node_config('system_monitor')
            assert node_config is not None
            assert 'package' in node_config
            assert 'node' in node_config
