#!/usr/bin/env python3
"""
Package Manager Utility
Reads package configurations from YAML files and installs packages
"""

import yaml
import subprocess
import sys
import argparse
from pathlib import Path
from typing import List, Dict, Any


class PackageManager:
    """Manages package installation from configuration files"""

    def __init__(self, config_dir: Path = None):
        if config_dir is None:
            config_dir = Path(__file__).parent.parent.parent / "config" / "system"
        self.config_dir = Path(config_dir)
        self.packages_config = self.config_dir / "packages.yaml"
        self.python_packages_config = self.config_dir / "python_packages.yaml"

    def load_config(self, config_file: Path) -> Dict[str, Any]:
        """Load YAML configuration file"""
        if not config_file.exists():
            raise FileNotFoundError(f"Config file not found: {config_file}")
        
        with open(config_file, 'r') as f:
            return yaml.safe_load(f) or {}

    def get_packages(self, groups: List[str] = None, categories: List[str] = None) -> List[str]:
        """Get list of packages from configuration"""
        config = self.load_config(self.packages_config)
        
        packages = []
        
        # If groups specified, resolve groups
        if groups:
            if 'groups' not in config:
                raise ValueError("No groups defined in configuration")
            
            for group in groups:
                if group not in config['groups']:
                    raise ValueError(f"Group '{group}' not found in configuration")
                
                # Resolve group dependencies
                group_categories = config['groups'][group]
                for category in group_categories:
                    if category in config['system']:
                        packages.extend(config['system'][category])
        
        # If categories specified, add those packages
        if categories:
            for category in categories:
                if category not in config['system']:
                    raise ValueError(f"Category '{category}' not found in configuration")
                packages.extend(config['system'][category])
        
        # Remove duplicates while preserving order
        seen = set()
        unique_packages = []
        for pkg in packages:
            if pkg not in seen:
                seen.add(pkg)
                unique_packages.append(pkg)
        
        return unique_packages

    def get_python_packages(self, groups: List[str] = None, categories: List[str] = None) -> List[str]:
        """Get list of Python packages from configuration"""
        config = self.load_config(self.python_packages_config)
        
        packages = []
        
        # If groups specified, resolve groups
        if groups:
            if 'groups' not in config:
                raise ValueError("No groups defined in configuration")
            
            for group in groups:
                if group not in config['groups']:
                    raise ValueError(f"Group '{group}' not found in configuration")
                
                # Resolve group dependencies
                group_categories = config['groups'][group]
                for category in group_categories:
                    if category in config:
                        packages.extend(config[category])
        
        # If categories specified, add those packages
        if categories:
            for category in categories:
                if category not in config:
                    raise ValueError(f"Category '{category}' not found in configuration")
                packages.extend(config[category])
        
        # Remove duplicates while preserving order
        seen = set()
        unique_packages = []
        for pkg in packages:
            if pkg not in seen:
                seen.add(pkg)
                unique_packages.append(pkg)
        
        return unique_packages

    def install_system_packages(self, packages: List[str], dry_run: bool = False) -> bool:
        """Install system packages using apt-get"""
        if not packages:
            print("No packages to install")
            return True
        
        cmd = ["sudo", "apt-get", "install", "-y"] + packages
        
        if dry_run:
            print(f"[DRY RUN] Would run: {' '.join(cmd)}")
            return True
        
        print(f"Installing {len(packages)} system packages...")
        try:
            result = subprocess.run(
                cmd,
                check=True,
                capture_output=False
            )
            print("System packages installed successfully")
            return True
        except subprocess.CalledProcessError as e:
            print(f"Error installing packages: {e}")
            return False

    def install_python_packages(self, packages: List[str], user: bool = True, dry_run: bool = False) -> bool:
        """Install Python packages using pip"""
        if not packages:
            print("No packages to install")
            return True
        
        cmd = ["pip3", "install", "--upgrade"]
        if user:
            cmd.append("--user")
        cmd.extend(packages)
        
        if dry_run:
            print(f"[DRY RUN] Would run: {' '.join(cmd)}")
            return True
        
        print(f"Installing {len(packages)} Python packages...")
        try:
            result = subprocess.run(
                cmd,
                check=True,
                capture_output=False
            )
            print("Python packages installed successfully")
            return True
        except subprocess.CalledProcessError as e:
            print(f"Error installing packages: {e}")
            return False

    def list_available(self, package_type: str = "system") -> None:
        """List available packages/groups"""
        if package_type == "system":
            config = self.load_config(self.packages_config)
            print("\nAvailable system package categories:")
            for category in config.get('system', {}).keys():
                count = len(config['system'][category])
                print(f"  - {category}: {count} packages")
            
            print("\nAvailable system package groups:")
            for group in config.get('groups', {}).keys():
                categories = config['groups'][group]
                print(f"  - {group}: {', '.join(categories)}")
        
        elif package_type == "python":
            config = self.load_config(self.python_packages_config)
            print("\nAvailable Python package categories:")
            for category in config.keys():
                if category != 'groups':
                    count = len(config[category])
                    print(f"  - {category}: {count} packages")
            
            print("\nAvailable Python package groups:")
            for group in config.get('groups', {}).keys():
                categories = config['groups'][group]
                print(f"  - {group}: {', '.join(categories)}")


def main():
    parser = argparse.ArgumentParser(description="Package Manager - Install packages from configuration")
    parser.add_argument(
        '--config-dir',
        type=Path,
        help='Configuration directory (default: config/system)'
    )
    
    subparsers = parser.add_subparsers(dest='command', help='Command to execute')
    
    # Install system packages
    install_sys = subparsers.add_parser('install-system', help='Install system packages')
    install_sys.add_argument(
        '--groups',
        nargs='+',
        help='Package groups to install'
    )
    install_sys.add_argument(
        '--categories',
        nargs='+',
        help='Package categories to install'
    )
    install_sys.add_argument(
        '--dry-run',
        action='store_true',
        help='Show what would be installed without installing'
    )
    
    # Install Python packages
    install_py = subparsers.add_parser('install-python', help='Install Python packages')
    install_py.add_argument(
        '--groups',
        nargs='+',
        help='Package groups to install'
    )
    install_py.add_argument(
        '--categories',
        nargs='+',
        help='Package categories to install'
    )
    install_py.add_argument(
        '--dry-run',
        action='store_true',
        help='Show what would be installed without installing'
    )
    install_py.add_argument(
        '--no-user',
        action='store_true',
        help='Install system-wide (requires sudo)'
    )
    
    # List available
    list_parser = subparsers.add_parser('list', help='List available packages/groups')
    list_parser.add_argument(
        'type',
        choices=['system', 'python'],
        help='Package type to list'
    )
    
    args = parser.parse_args()
    
    if not args.command:
        parser.print_help()
        sys.exit(1)
    
    pm = PackageManager(args.config_dir)
    
    if args.command == 'install-system':
        try:
            packages = pm.get_packages(groups=args.groups, categories=args.categories)
            if packages:
                print(f"Found {len(packages)} packages to install")
                pm.install_system_packages(packages, dry_run=args.dry_run)
            else:
                print("No packages found")
        except Exception as e:
            print(f"Error: {e}", file=sys.stderr)
            sys.exit(1)
    
    elif args.command == 'install-python':
        try:
            packages = pm.get_python_packages(groups=args.groups, categories=args.categories)
            if packages:
                print(f"Found {len(packages)} packages to install")
                pm.install_python_packages(packages, user=not args.no_user, dry_run=args.dry_run)
            else:
                print("No packages found")
        except Exception as e:
            print(f"Error: {e}", file=sys.stderr)
            sys.exit(1)
    
    elif args.command == 'list':
        pm.list_available(args.type)


if __name__ == '__main__':
    main()

