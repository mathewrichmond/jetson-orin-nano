#!/bin/bash
# Identify which physical RealSense camera is front vs rear

set -e

echo "=========================================="
echo "RealSense Camera Identification"
echo "=========================================="
echo ""

# Check detected cameras
echo "Detected RealSense cameras:"
rs-enumerate-devices -s | grep -E "Serial Number|Device Name" | head -6
echo ""

# Check ROS topics to see which camera is publishing
echo "Checking ROS 2 topics..."
if command -v ros2 &> /dev/null; then
    # Source ROS 2 if available
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash 2>/dev/null || true
    fi
    if [ -f ~/ros2_ws/install/setup.bash ]; then
        source ~/ros2_ws/install/setup.bash 2>/dev/null || true
    fi
    
    echo ""
    echo "Camera topic status:"
    
    # Check front camera
    if timeout 2 ros2 topic echo /hardware/camera_front/color/image_raw --once --field header 2>/dev/null > /dev/null; then
        echo "  ✓ camera_front: PUBLISHING (serial: check below)"
        FRONT_ACTIVE=true
    else
        echo "  ✗ camera_front: NOT PUBLISHING"
        FRONT_ACTIVE=false
    fi
    
    # Check rear camera
    if timeout 2 ros2 topic echo /hardware/camera_rear/color/image_raw --once --field header 2>/dev/null > /dev/null; then
        echo "  ✓ camera_rear: PUBLISHING (serial: check below)"
        REAR_ACTIVE=true
    else
        echo "  ✗ camera_rear: NOT PUBLISHING"
        REAR_ACTIVE=false
    fi
    
    echo ""
    echo "To identify physical cameras:"
    echo "1. Unplug ONE camera USB cable"
    echo "2. Run this script again"
    echo "3. The camera that disappears is the one you unplugged"
    echo ""
    echo "Or check the camera_info topics for serial numbers:"
    echo "  ros2 topic echo /hardware/camera_front/color/camera_info --once | grep -A 5 frame_id"
    echo "  ros2 topic echo /hardware/camera_rear/color/camera_info --once | grep -A 5 frame_id"
else
    echo "ROS 2 not available - cannot check topics"
fi

echo ""
echo "Current camera assignment (from config):"
if [ -f "/home/nano/src/jetson-orin-nano/config/robot/robot_graph.yaml" ]; then
    grep -A 2 "camera_names:" /home/nano/src/jetson-orin-nano/config/robot/robot_graph.yaml | head -3
fi
