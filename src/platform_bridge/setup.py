#!/usr/bin/env python3
"""Platform bridge setup script."""

# Third-party
from setuptools import find_packages, setup

setup(
    name="platform_bridge",
    version="0.1.0",
    description="Jetson robot integration with cross-platform robotics system",
    author="Your Name",
    author_email="your.email@example.com",
    packages=find_packages(),
    install_requires=[
        "pyyaml",
        "numpy",
        "eclipse-zenoh",
        # Note: rclpy is installed via ROS 2, not pip
    ],
    extras_require={
        "compression": [
            "zstandard",
            "lz4",
        ],
    },
    entry_points={
        "console_scripts": [
            "platform_bridge=platform_bridge.jetson_robot:main",
        ],
    },
    python_requires=">=3.8",
)
