#!/usr/bin/env python3
"""
Hardware Manager Utility
Manages hardware component installation and configuration
"""

import subprocess
import sys
import argparse
import os
import shutil
from pathlib import Path
from typing import List, Dict, Any, Optional
import json


class HardwareManager:
    """Manages hardware component installation and configuration"""

    def __init__(self, project_root: Path = None):
        if project_root is None:
            project_root = Path(__file__).parent.parent.parent
        self.project_root = Path(project_root)
        self.scripts_dir = self.project_root / "scripts" / "hardware"
        self.state_file = self.project_root / ".hardware_state"

    def load_state(self) -> Dict[str, Any]:
        """Load hardware installation state"""
        if self.state_file.exists():
            try:
                with open(self.state_file, 'r') as f:
                    return json.load(f)
            except Exception:
                return {}
        return {}

    def save_state(self, state: Dict[str, Any]):
        """Save hardware installation state"""
        with open(self.state_file, 'w') as f:
            json.dump(state, f, indent=2)

    def is_installed(self, component: str) -> bool:
        """Check if a hardware component is installed"""
        state = self.load_state()
        return state.get(component, {}).get('installed', False)

    def mark_installed(self, component: str, details: Dict[str, Any] = None):
        """Mark a hardware component as installed"""
        state = self.load_state()
        if component not in state:
            state[component] = {}
        state[component]['installed'] = True
        if details:
            state[component].update(details)
        self.save_state(state)

    def check_root(self) -> bool:
        """Check if running as root"""
        return os.geteuid() == 0

    def require_root(self):
        """Require root privileges or exit"""
        if not self.check_root():
            print("ERROR: This operation requires root privileges.")
            print("Please run with sudo or as root.")
            sys.exit(1)

    def detect_environment(self) -> str:
        """Detect the environment type"""
        if os.path.exists('/.dockerenv') or os.path.exists('/proc/1/cgroup'):
            with open('/proc/1/cgroup', 'r') as f:
                if 'docker' in f.read():
                    return 'docker'
        if os.path.exists('/etc/nv_tegra_release'):
            return 'jetson'
        if os.path.exists('/etc/os-release'):
            with open('/etc/os-release', 'r') as f:
                if 'Ubuntu' in f.read():
                    return 'ubuntu'
        return 'unknown'

    def install_realsense(self, skip_if_installed: bool = True) -> bool:
        """Install Intel RealSense SDK and ROS 2 wrapper"""
        if skip_if_installed and self.is_installed('realsense'):
            print("RealSense SDK already installed (skipping)")
            return True

        self.require_root()

        print("==========================================")
        print("Installing Intel RealSense SDK")
        print("==========================================")

        # Update package list
        print("Updating package list...")
        subprocess.run(['apt-get', 'update'], check=True)

        # Install dependencies
        print("Installing dependencies...")
        deps = [
            'git', 'libssl-dev', 'libusb-1.0-0-dev', 'pkg-config',
            'libgtk-3-dev', 'libglfw3-dev', 'libgl1-mesa-dev',
            'libglu1-mesa-dev', 'python3-dev', 'python3-pip',
            'cmake', 'build-essential'
        ]
        subprocess.run(['apt-get', 'install', '-y'] + deps, check=True)

        # Install pyrealsense2 via pip
        print("Installing pyrealsense2...")
        try:
            subprocess.run(['pip3', 'install', 'pyrealsense2'], check=True)
        except subprocess.CalledProcessError:
            # Try with --user flag
            subprocess.run(['pip3', 'install', '--user', 'pyrealsense2'], check=True)

        # Build and install librealsense from source
        print("Building librealsense from source...")
        realsense_dir = Path('/tmp/librealsense')
        if realsense_dir.exists():
            shutil.rmtree(realsense_dir)

        # Clone repository
        subprocess.run([
            'git', 'clone', 'https://github.com/IntelRealSense/librealsense.git',
            str(realsense_dir)
        ], check=True)

        # Checkout stable version
        subprocess.run([
            'git', '-C', str(realsense_dir), 'checkout', 'v2.54.2'
        ], check=True)

        # Build
        build_dir = realsense_dir / 'build'
        build_dir.mkdir(exist_ok=True)

        cmake_cmd = [
            'cmake', '..',
            '-DCMAKE_BUILD_TYPE=Release',
            '-DBUILD_EXAMPLES=true',
            '-DBUILD_GRAPHICAL_EXAMPLES=false',
            '-DBUILD_PYTHON_BINDINGS=true',
            '-DPYTHON_EXECUTABLE=/usr/bin/python3'
        ]
        subprocess.run(cmake_cmd, cwd=str(build_dir), check=True)

        # Build (this may take a while)
        print("Building librealsense (this may take 10-20 minutes)...")
        import multiprocessing
        num_cores = multiprocessing.cpu_count()
        subprocess.run(['make', '-j', str(num_cores)], cwd=str(build_dir), check=True)

        # Install
        subprocess.run(['make', 'install'], cwd=str(build_dir), check=True)
        subprocess.run(['ldconfig'], check=True)

        # Install udev rules
        print("Installing udev rules...")
        udev_rules = realsense_dir / 'config' / '99-realsense-libusb.rules'
        if udev_rules.exists():
            shutil.copy(udev_rules, '/etc/udev/rules.d/')
            subprocess.run(['udevadm', 'control', '--reload-rules'], check=True)
            subprocess.run(['udevadm', 'trigger'], check=True)

        # Install ROS 2 wrapper if ROS 2 is installed
        ros2_installed = os.path.exists('/opt/ros/humble/setup.bash')
        if ros2_installed:
            print("Installing ROS 2 RealSense wrapper...")
            ros2_ws = Path.home() / 'ros2_ws'
            ros2_ws_src = ros2_ws / 'src'
            ros2_ws_src.mkdir(parents=True, exist_ok=True)

            wrapper_dir = ros2_ws_src / 'realsense-ros'
            if not wrapper_dir.exists():
                subprocess.run([
                    'git', 'clone',
                    'https://github.com/IntelRealSense/realsense-ros.git',
                    '-b', 'ros2-development',
                    str(wrapper_dir)
                ], check=True)

            # Install dependencies
            if shutil.which('rosdep'):
                subprocess.run([
                    'bash', '-c',
                    'source /opt/ros/humble/setup.bash && '
                    'rosdep install --from-paths src --ignore-src -r -y || true'
                ], cwd=str(ros2_ws), check=False)

            print("ROS 2 wrapper cloned. Build with:")
            print(f"  cd {ros2_ws}")
            print("  source /opt/ros/humble/setup.bash")
            print("  colcon build --packages-select realsense2_camera")

        # Mark as installed
        self.mark_installed('realsense', {
            'version': '2.54.2',
            'ros2_wrapper': ros2_installed
        })

        print("")
        print("==========================================")
        print("RealSense SDK installation complete!")
        print("==========================================")
        print("")
        print("To test installation:")
        print("  realsense-viewer")
        print("")
        print("To verify cameras:")
        print("  rs-enumerate-devices")
        print("")
        print("To add user to dialout group (for USB access):")
        print("  sudo usermod -a -G dialout $USER")
        print("  (then log out and back in)")

        return True

    def build_realsense_ros_package(self, skip_if_built: bool = True) -> bool:
        """Build the RealSense ROS 2 package"""
        if skip_if_built and self.is_installed('realsense_ros_package'):
            print("RealSense ROS package already built (skipping)")
            return True

        # Check if ROS 2 is installed
        if not os.path.exists('/opt/ros/humble/setup.bash'):
            print("ERROR: ROS 2 Humble not found")
            return False

        # Check if workspace exists
        ros2_ws = Path.home() / 'ros2_ws'
        if not ros2_ws.exists():
            print("ERROR: ROS 2 workspace not found")
            print("Run setup.sh first to create workspace")
            return False

        print("Building RealSense ROS 2 package...")

        # Source ROS 2 and build
        build_cmd = [
            'bash', '-c',
            'source /opt/ros/humble/setup.bash && '
            f'cd {ros2_ws} && '
            'colcon build --packages-select realsense_camera'
        ]

        try:
            subprocess.run(build_cmd, check=True)
            self.mark_installed('realsense_ros_package', {
                'workspace': str(ros2_ws)
            })
            print("RealSense ROS package built successfully!")
            return True
        except subprocess.CalledProcessError as e:
            print(f"ERROR: Failed to build RealSense ROS package: {e}")
            return False

    def diagnose_realsense(self) -> bool:
        """Run RealSense camera diagnostics"""
        script = self.scripts_dir / 'diagnose_realsense.sh'
        if not script.exists():
            print(f"ERROR: Diagnostic script not found: {script}")
            return False

        try:
            subprocess.run(['bash', str(script)], check=True)
            return True
        except subprocess.CalledProcessError:
            return False

    def list_components(self):
        """List available hardware components"""
        components = {
            'realsense': {
                'name': 'Intel RealSense Cameras',
                'description': 'D435/D455 depth cameras',
                'installed': self.is_installed('realsense')
            }
        }

        print("\nAvailable hardware components:")
        for comp_id, comp_info in components.items():
            status = "✓ Installed" if comp_info['installed'] else "✗ Not installed"
            print(f"\n  {comp_info['name']} ({comp_id})")
            print(f"    Description: {comp_info['description']}")
            print(f"    Status: {status}")


def main():
    parser = argparse.ArgumentParser(
        description="Hardware Manager - Install and manage hardware components"
    )

    subparsers = parser.add_subparsers(dest='command', help='Command to execute')

    # Install RealSense
    install_rs = subparsers.add_parser('install-realsense', help='Install RealSense SDK')
    install_rs.add_argument(
        '--force',
        action='store_true',
        help='Reinstall even if already installed'
    )

    # Build RealSense ROS package
    build_rs = subparsers.add_parser('build-realsense-ros', help='Build RealSense ROS package')
    build_rs.add_argument(
        '--force',
        action='store_true',
        help='Rebuild even if already built'
    )

    # Diagnose RealSense
    diag_rs = subparsers.add_parser('diagnose-realsense', help='Run RealSense diagnostics')

    # List components
    list_parser = subparsers.add_parser('list', help='List available components')

    args = parser.parse_args()

    if not args.command:
        parser.print_help()
        sys.exit(1)

    manager = HardwareManager()

    if args.command == 'install-realsense':
        success = manager.install_realsense(skip_if_installed=not args.force)
        sys.exit(0 if success else 1)

    elif args.command == 'build-realsense-ros':
        success = manager.build_realsense_ros_package(skip_if_built=not args.force)
        sys.exit(0 if success else 1)

    elif args.command == 'diagnose-realsense':
        success = manager.diagnose_realsense()
        sys.exit(0 if success else 1)

    elif args.command == 'list':
        manager.list_components()


if __name__ == '__main__':
    main()
