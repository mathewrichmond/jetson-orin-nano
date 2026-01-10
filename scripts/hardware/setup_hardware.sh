#!/bin/bash
# Unified Hardware Setup Script
# Handles all hardware installation, verification, and configuration
# Integrates with unified setup system

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
UTILS_DIR="${PROJECT_ROOT}/scripts/utils"

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m'

usage() {
    cat << EOF
Unified Hardware Setup and Management

Usage: $0 <command> [options]

Commands:
  install [component]    Install hardware component(s)
  verify                 Verify all hardware is connected and working
  diagnose [component]  Run diagnostics for specific component
  list                   List available hardware components
  status                 Show hardware status

Components:
  realsense             RealSense cameras
  microphone            USB microphone
  odrive                ODrive motor controller
  irobot                iRobot Create/Roomba
  all                   All components

Examples:
  $0 install realsense
  $0 verify
  $0 diagnose realsense
  $0 status

EOF
}

install_component() {
    local component="${1:-}"

    case "$component" in
        realsense)
            echo -e "${GREEN}Installing RealSense cameras...${NC}"
            sudo "${SCRIPT_DIR}/install_realsense.sh"
            ;;
        microphone)
            echo -e "${GREEN}Installing USB microphone support...${NC}"
            sudo apt-get update
            sudo apt-get install -y alsa-utils pulseaudio
            ;;
        odrive)
            echo -e "${GREEN}Installing ODrive support...${NC}"
            sudo apt-get update
            sudo apt-get install -y python3-pip python3-serial
            pip3 install pyserial
            ;;
        irobot)
            echo -e "${GREEN}Installing iRobot support...${NC}"
            sudo apt-get update
            sudo apt-get install -y python3-pip python3-serial
            pip3 install pyserial
            ;;
        all)
            install_component realsense
            install_component microphone
            install_component odrive
            install_component irobot
            ;;
        *)
            echo -e "${RED}Error: Unknown component '$component'${NC}"
            usage
            exit 1
            ;;
    esac
}

verify_hardware() {
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}Hardware Verification${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo ""

    "${SCRIPT_DIR}/verify_all_hardware.sh"
}

diagnose_component() {
    local component="${1:-}"

    case "$component" in
        realsense)
            "${SCRIPT_DIR}/diagnose_realsense.sh"
            ;;
        *)
            echo -e "${RED}Error: Diagnostics not available for '$component'${NC}"
            echo "Available: realsense"
            exit 1
            ;;
    esac
}

show_status() {
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}Hardware Status${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo ""

    # Check USB devices
    echo "USB Devices:"
    lsusb | head -10
    echo ""

    # Check serial devices
    echo "Serial Devices:"
    ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "  None found"
    echo ""

    # Check audio devices
    if command -v pactl &> /dev/null; then
        echo "Audio Devices:"
        pactl list sources short 2>/dev/null | head -5 || echo "  None found"
    fi
    echo ""

    # Check ROS 2 topics (if ROS 2 is available)
    if command -v ros2 &> /dev/null && ros2 topic list &>/dev/null 2>&1; then
        echo "ROS 2 Hardware Topics:"
        ros2 topic list 2>/dev/null | grep -E "(camera|microphone|odrive|irobot|system)" | head -10 || echo "  No hardware topics found"
    fi
}

list_components() {
    echo -e "${BLUE}Available Hardware Components:${NC}"
    echo ""
    echo "  realsense  - Intel RealSense cameras (D435/D455)"
    echo "  microphone - USB microphone"
    echo "  odrive     - ODrive motor controller"
    echo "  irobot     - iRobot Create/Roomba"
    echo ""
}

# Main command handling
case "${1:-}" in
    install)
        install_component "${2:-all}"
        ;;
    verify)
        verify_hardware
        ;;
    diagnose)
        diagnose_component "${2:-}"
        ;;
    status)
        show_status
        ;;
    list)
        list_components
        ;;
    *)
        usage
        exit 1
        ;;
esac
