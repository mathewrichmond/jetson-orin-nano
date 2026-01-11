"""
Pytest configuration and shared fixtures for unified testing framework
"""

# Standard library
from pathlib import Path
import sys

# Third-party
import pytest

# Add src to path for imports
REPO_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(REPO_ROOT / "src"))


@pytest.fixture(scope="session")
def repo_root():
    """Return repository root directory"""
    return REPO_ROOT


@pytest.fixture(scope="session")
def ros2_workspace():
    """Return ROS 2 workspace path"""
    # Standard library
    import os

    return Path(os.path.expanduser("~/ros2_ws"))


@pytest.fixture(scope="session")
def isaac_root():
    """Return Isaac root directory (dev or installed)"""
    dev_dir = Path("/home/nano/src/jetson-orin-nano")
    install_dir = Path("/opt/isaac-robot")

    if dev_dir.exists():
        return dev_dir
    elif install_dir.exists():
        return install_dir
    else:
        return Path.cwd()
