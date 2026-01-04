#!/bin/bash
# Install Intel RealSense SDK and ROS 2 wrapper
# This script installs the RealSense SDK 2.0 and the ROS 2 wrapper for Jetson

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Check for root privileges
if [ "$EUID" -ne 0 ]; then
    echo "Please run as root (use sudo)"
    exit 1
fi

echo "=========================================="
echo "Installing Intel RealSense SDK"
echo "=========================================="

# Update package list
apt-get update

# Install dependencies
echo "Installing dependencies..."
apt-get install -y \
    git \
    libssl-dev \
    libusb-1.0-0-dev \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    python3-dev \
    python3-pip \
    cmake \
    build-essential

# Install pyrealsense2 via pip
echo "Installing pyrealsense2..."
pip3 install pyrealsense2

# Build and install librealsense from source (for Jetson compatibility)
echo "Building librealsense from source..."
REALSENSE_DIR="/tmp/librealsense"
if [ -d "$REALSENSE_DIR" ]; then
    rm -rf "$REALSENSE_DIR"
fi

git clone https://github.com/IntelRealSense/librealsense.git "$REALSENSE_DIR"
cd "$REALSENSE_DIR"

# Checkout stable version
git checkout v2.54.2

# Configure for Jetson
mkdir build && cd build
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_EXAMPLES=true \
    -DBUILD_GRAPHICAL_EXAMPLES=false \
    -DBUILD_PYTHON_BINDINGS=true \
    -DPYTHON_EXECUTABLE=/usr/bin/python3

# Build (this may take a while)
echo "Building librealsense (this may take 10-20 minutes)..."
make -j$(nproc)

# Install
make install
ldconfig

# Install udev rules
echo "Installing udev rules..."
cp "$REALSENSE_DIR/config/99-realsense-libusb.rules" /etc/udev/rules.d/
udevadm control --reload-rules
udevadm trigger

# Install ROS 2 wrapper if ROS 2 is installed
if [ -f "/opt/ros/humble/setup.bash" ]; then
    echo "Installing ROS 2 RealSense wrapper..."

    ROS2_WS="${ROS2_WS:-$HOME/ros2_ws}"
    if [ ! -d "$ROS2_WS/src" ]; then
        mkdir -p "$ROS2_WS/src"
    fi

    cd "$ROS2_WS/src"

    # Clone realsense-ros if not already present
    if [ ! -d "realsense-ros" ]; then
        git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-development
    fi

    # Install dependencies
    cd "$ROS2_WS"
    if command -v rosdep &> /dev/null; then
        source /opt/ros/humble/setup.bash
        rosdep install --from-paths src --ignore-src -r -y || true
    fi

    echo "ROS 2 wrapper cloned. Build with:"
    echo "  cd $ROS2_WS"
    echo "  source /opt/ros/humble/setup.bash"
    echo "  colcon build --packages-select realsense2_camera"
fi

echo ""
echo "=========================================="
echo "RealSense SDK installation complete!"
echo "=========================================="
echo ""
echo "To test installation:"
echo "  realsense-viewer"
echo ""
echo "To verify cameras:"
echo "  rs-enumerate-devices"
echo ""
echo "To add user to dialout group (for USB access):"
echo "  sudo usermod -a -G dialout $USER"
echo "  (then log out and back in)"
