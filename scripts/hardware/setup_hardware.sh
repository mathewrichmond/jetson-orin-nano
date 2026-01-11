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
  phat                  PHAT motor controller (GPIO + accelerometer)
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
            sudo apt-get install -y alsa-utils pulseaudio alsa-base

            # Ensure user is in audio group
            if ! groups $USER | grep -q "\baudio\b"; then
                echo -e "${YELLOW}Adding user to audio group...${NC}"
                sudo usermod -a -G audio $USER
                echo -e "${YELLOW}Note: You may need to log out and back in for audio group changes to take effect${NC}"
            fi

            # Reload ALSA modules
            echo -e "${BLUE}Reloading ALSA modules...${NC}"
            sudo modprobe snd-usb-audio || true

            # Test microphone if available
            if arecord -l | grep -q "USB Audio"; then
                echo -e "${BLUE}Testing microphone...${NC}"
                MIC_CARD=$(arecord -l | grep "USB Audio" | head -1 | sed 's/^card \([0-9]*\):.*/\1/')
                if [ -n "$MIC_CARD" ]; then
                    echo -e "${BLUE}Found USB microphone on card $MIC_CARD${NC}"
                    # Test recording (2 seconds, stereo, 16kHz)
                    if timeout 3 arecord -D plughw:$MIC_CARD,0 -d 2 -f S16_LE -r 16000 -c 2 /tmp/mic_test.wav 2>/dev/null; then
                        echo -e "${GREEN}Microphone test recording successful${NC}"
                        rm -f /tmp/mic_test.wav
                    else
                        echo -e "${YELLOW}Microphone test recording failed - check device configuration${NC}"
                    fi
                fi
            else
                echo -e "${YELLOW}No USB microphone detected - connect microphone and run verification${NC}"
            fi

            echo -e "${GREEN}USB microphone support installed${NC}"
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
        phat)
            echo -e "${GREEN}Installing PHAT motor controller support...${NC}"
            sudo apt-get update
            sudo apt-get install -y python3-pip i2c-tools
            pip3 install smbus2
            echo "  Note: User will be added to i2c group for I2C access"
            echo "  Note: Jetson.GPIO should already be installed for GPIO access"
            ;;
        all)
            install_component realsense
            install_component microphone
            install_component odrive
            install_component irobot
            install_component phat
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
    echo "  phat       - PHAT motor controller (GPIO + accelerometer)"
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
