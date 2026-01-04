#!/usr/bin/env python3
"""
Find Isaac Robot Root Directory
Checks dev sandbox first, then installed package
"""

import os
import sys
from pathlib import Path

DEV_DIR = Path.home() / "src" / "jetson-orin-nano"
INSTALL_DIR = Path("/opt/isaac-robot")

def find_isaac_root():
    """Find Isaac root directory with dev priority"""
    # Check dev directory first
    if DEV_DIR.exists() and (DEV_DIR / "setup.sh").exists():
        return str(DEV_DIR)

    # Check installed package
    if INSTALL_DIR.exists() and (INSTALL_DIR / "setup.sh").exists():
        return str(INSTALL_DIR)

    # Check ISAAC_DEV_DIR environment variable
    dev_env = os.getenv("ISAAC_DEV_DIR")
    if dev_env and Path(dev_env).exists() and Path(dev_env / "setup.sh").exists():
        return dev_env

    # Check ISAAC_INSTALL_DIR environment variable
    install_env = os.getenv("ISAAC_INSTALL_DIR")
    if install_env and Path(install_env).exists() and Path(install_env / "setup.sh").exists():
        return install_env

    return None

if __name__ == "__main__":
    root = find_isaac_root()
    if root:
        print(root)
        sys.exit(0)
    else:
        print("Error: Isaac robot root not found", file=sys.stderr)
        sys.exit(1)
