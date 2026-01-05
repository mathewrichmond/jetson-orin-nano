#!/usr/bin/env python3
"""
Config Finder Utility
Finds configuration files in the centralized config/ directory
Supports both dev and installed environments
"""

import os
from pathlib import Path
from typing import Optional


def find_config_file(config_name: str, category: str = 'hardware') -> Optional[Path]:
    """
    Find a configuration file in the centralized config/ directory

    Args:
        config_name: Name of config file (e.g., 'realsense_params.yaml')
        category: Config category ('hardware', 'system', 'robot', 'control')

    Returns:
        Path to config file, or None if not found
    """
    # Try to find Isaac root
    isaac_root = find_isaac_root()
    if not isaac_root:
        return None

    config_path = isaac_root / 'config' / category / config_name

    if config_path.exists():
        return config_path

    return None


def find_isaac_root() -> Optional[Path]:
    """
    Find the Isaac root directory (dev or installed)

    Returns:
        Path to Isaac root, or None if not found
    """
    # Try dev location first
    dev_dir = Path('/home/nano/src/jetson-orin-nano')
    if dev_dir.exists() and (dev_dir / 'config').exists():
        return dev_dir

    # Try installed location
    install_dir = Path('/opt/isaac-robot')
    if install_dir.exists() and (install_dir / 'config').exists():
        return install_dir

    # Try current working directory
    cwd = Path.cwd()
    if (cwd / 'config').exists():
        return cwd

    # Try relative to this file
    this_file = Path(__file__)
    repo_root = this_file.parent.parent.parent.parent
    if (repo_root / 'config').exists():
        return repo_root

    return None


def get_config_path(config_name: str, category: str = 'hardware',
                    fallback_to_package: bool = True) -> str:
    """
    Get config file path, with fallback to package config if needed

    Args:
        config_name: Name of config file
        category: Config category
        fallback_to_package: If True, fall back to package config directory

    Returns:
        Path to config file as string
    """
    config_path = find_config_file(config_name, category)

    if config_path:
        return str(config_path)

    # Fallback to package config (for backward compatibility)
    if fallback_to_package:
        # Try to find in package share
        try:
            from ament_index_python.packages import get_package_share_directory
            # Try common package names
            for pkg in ['isaac_robot', 'realsense_camera', 'system_monitor']:
                try:
                    pkg_share = get_package_share_directory(pkg)
                    pkg_config = Path(pkg_share) / 'config' / config_name
                    if pkg_config.exists():
                        return str(pkg_config)
                except:
                    continue
        except:
            pass

    # Last resort: return config name and let ROS 2 handle it
    return config_name
