"""
Pytest Configuration and Shared Fixtures

This file provides common fixtures and configuration for all tests.
"""

import os
import sys
from pathlib import Path
from typing import Generator

import pytest

# Add src to path for imports
REPO_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))


@pytest.fixture(scope="session")
def repo_root() -> Path:
    """Get repository root directory"""
    return REPO_ROOT


@pytest.fixture(scope="session")
def test_data_dir(repo_root: Path) -> Path:
    """Get test data directory"""
    return repo_root / "tests" / "fixtures"


@pytest.fixture(scope="session")
def config_dir(repo_root: Path) -> Path:
    """Get config directory"""
    return repo_root / "config"


@pytest.fixture
def ros_domain_id() -> Generator[int, None, None]:
    """Set isolated ROS domain for testing"""
    original = os.environ.get("ROS_DOMAIN_ID")
    test_domain = "42"  # Isolated test domain
    os.environ["ROS_DOMAIN_ID"] = test_domain
    
    yield int(test_domain)
    
    # Restore original
    if original is not None:
        os.environ["ROS_DOMAIN_ID"] = original
    else:
        os.environ.pop("ROS_DOMAIN_ID", None)


@pytest.fixture
def mock_hardware_mode() -> Generator[bool, None, None]:
    """Enable mock hardware mode for testing"""
    original = os.environ.get("MOCK_HARDWARE")
    os.environ["MOCK_HARDWARE"] = "true"
    
    yield True
    
    # Restore original
    if original is not None:
        os.environ["MOCK_HARDWARE"] = original
    else:
        os.environ.pop("MOCK_HARDWARE", None)


# Test markers
def pytest_configure(config):
    """Register custom markers"""
    config.addinivalue_line(
        "markers", "unit: Unit tests (hermetic, fast)"
    )
    config.addinivalue_line(
        "markers", "integration: Integration tests (require ROS 2)"
    )
    config.addinivalue_line(
        "markers", "hardware: Hardware tests (require real devices)"
    )
    config.addinivalue_line(
        "markers", "slow: Slow tests (> 1 second)"
    )
    config.addinivalue_line(
        "markers", "gpu: GPU tests (require CUDA)"
    )


# Test collection
def pytest_collection_modifyitems(config, items):
    """Modify test collection based on markers"""
    # Skip hardware tests by default
    skip_hardware = pytest.mark.skip(reason="Requires hardware (use --hardware to run)")
    skip_gpu = pytest.mark.skip(reason="Requires GPU (use --gpu to run)")
    
    for item in items:
        if "hardware" in item.keywords and not config.getoption("--hardware", default=False):
            item.add_marker(skip_hardware)
        if "gpu" in item.keywords and not config.getoption("--gpu", default=False):
            item.add_marker(skip_gpu)


def pytest_addoption(parser):
    """Add custom command line options"""
    parser.addoption(
        "--hardware",
        action="store_true",
        default=False,
        help="Run hardware tests"
    )
    parser.addoption(
        "--gpu",
        action="store_true",
        default=False,
        help="Run GPU tests"
    )
    parser.addoption(
        "--ros-domain",
        action="store",
        default="42",
        help="ROS domain ID for tests"
    )
