#!/bin/bash
# Unified Hardware Setup Script
# Single entry point for all Jetson hardware installation, configuration, and verification.
#
# Usage:
#   ./scripts/hardware/setup_hardware.sh full-setup     # Run everything (first-time setup)
#   ./scripts/hardware/setup_hardware.sh install all    # Install component drivers
#   ./scripts/hardware/setup_hardware.sh verify         # Verify all hardware
#   ./scripts/hardware/setup_hardware.sh status         # Quick status check
#   ./scripts/hardware/setup_hardware.sh diagnose <component>

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
RED='\033[0;31m'
CYAN='\033[0;36m'
NC='\033[0m'

step()  { echo -e "\n${CYAN}==> $*${NC}"; }
ok()    { echo -e "    ${GREEN}[OK]${NC} $*"; }
warn()  { echo -e "    ${YELLOW}[WARN]${NC} $*"; }
info()  { echo -e "    ${BLUE}$*${NC}"; }
fail()  { echo -e "    ${RED}[FAIL]${NC} $*"; exit 1; }

usage() {
    cat << EOF
Unified Hardware Setup and Management

Usage: $0 <command> [options]

Commands:
  full-setup             Run complete hardware setup (first-time use after flash)
  install [component]    Install hardware component driver(s)
  verify                 Verify all hardware is connected and working
  diagnose [component]   Run diagnostics for specific component
  list                   List available hardware components
  status                 Show hardware status

Components (for install/diagnose):
  realsense              Intel RealSense cameras
  microphone             USB microphone
  odrive                 ODrive motor controller
  irobot                 iRobot Create/Roomba
  phat                   SparkFun Auto pHAT (PCA9685 servo + ICM-20948 IMU)
  all                    All components

full-setup runs in two passes (a reboot is required between them):
  Pass 1: Groups, udev rules, I2C overlay (patches extlinux.conf) → REBOOT
  Pass 2 (after reboot): Verifies I2C, installs camera/audio/GPIO drivers → DONE

Examples:
  $0 full-setup
  $0 install realsense
  $0 verify
  $0 diagnose phat
  $0 status

EOF
}

# ─────────────────────────────────────────────────────────────────────────────
# full-setup helpers
# ─────────────────────────────────────────────────────────────────────────────

setup_system_groups() {
    step "Configuring system groups for hardware access"

    for grp in i2c gpio dialout audio video; do
        if getent group "$grp" &>/dev/null; then
            if ! groups "$USER" | grep -qw "$grp"; then
                sudo usermod -aG "$grp" "$USER"
                ok "Added $USER to $grp group"
            else
                ok "$USER already in $grp group"
            fi
        else
            sudo groupadd "$grp"
            sudo usermod -aG "$grp" "$USER"
            ok "Created $grp group and added $USER"
        fi
    done

    warn "Group changes take effect on next login (or run: newgrp <group>)"
}

install_udev_rules() {
    step "Installing udev rules for container hardware access"

    local rules_src="${PROJECT_ROOT}/config/hardware/udev/99-robot-hardware.rules"
    local rules_dst="/etc/udev/rules.d/99-robot-hardware.rules"

    if [ ! -f "$rules_src" ]; then
        fail "udev rules not found: $rules_src"
    fi

    if diff -q "$rules_src" "$rules_dst" &>/dev/null 2>&1; then
        ok "udev rules already up to date"
    else
        sudo cp "$rules_src" "$rules_dst"
        sudo udevadm control --reload-rules
        sudo udevadm trigger
        ok "udev rules installed and reloaded"
    fi
}

# ─────────────────────────────────────────────────────────────────────────────
# Component install
# ─────────────────────────────────────────────────────────────────────────────

install_component() {
    local component="${1:-}"

    case "$component" in
        realsense)
            step "Installing RealSense cameras"
            sudo "${SCRIPT_DIR}/install_realsense.sh"
            ;;
        microphone)
            step "Installing USB microphone support"
            sudo apt-get update -qq
            sudo apt-get install -y alsa-utils pulseaudio alsa-base

            if ! groups "$USER" | grep -qw "audio"; then
                sudo usermod -aG audio "$USER"
                warn "Added $USER to audio group — log out and back in for it to take effect"
            fi

            sudo modprobe snd-usb-audio 2>/dev/null || true

            if arecord -l 2>/dev/null | grep -q "USB Audio"; then
                MIC_CARD=$(arecord -l | grep "USB Audio" | head -1 | sed 's/^card \([0-9]*\):.*/\1/')
                if [ -n "$MIC_CARD" ]; then
                    info "USB microphone on card $MIC_CARD"
                    if timeout 3 arecord -D "plughw:$MIC_CARD,0" -d 2 -f S16_LE -r 16000 -c 2 /tmp/mic_test.wav 2>/dev/null; then
                        ok "Microphone test recording successful"
                        rm -f /tmp/mic_test.wav
                    else
                        warn "Microphone test recording failed — check device config"
                    fi
                fi
            else
                warn "No USB microphone detected — connect and re-run verify"
            fi
            ok "USB microphone support installed"
            ;;
        odrive)
            step "Installing ODrive support"
            sudo apt-get update -qq
            sudo apt-get install -y python3-pip python3-serial
            pip3 install --user pyserial
            ok "ODrive support installed"
            ;;
        irobot)
            step "Installing iRobot support"
            sudo apt-get update -qq
            sudo apt-get install -y python3-pip python3-serial
            pip3 install --user pyserial
            ok "iRobot support installed"
            ;;
        phat)
            step "Installing SparkFun Auto pHAT support"
            sudo apt-get update -qq
            sudo apt-get install -y python3-pip i2c-tools
            pip3 install --user smbus2
            info "i2c-tools installed; I2C setup handled by setup_i2c.sh"
            ok "Auto pHAT support installed"
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

# ─────────────────────────────────────────────────────────────────────────────
# verify / diagnose / status / list
# ─────────────────────────────────────────────────────────────────────────────

verify_hardware() {
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}Hardware Verification${NC}"
    echo -e "${BLUE}========================================${NC}"
    "${SCRIPT_DIR}/verify_all_hardware.sh"
}

diagnose_component() {
    local component="${1:-}"
    case "$component" in
        phat)
            "${SCRIPT_DIR}/diagnose_phat.sh"
            ;;
        pca9685|servo)
            "${SCRIPT_DIR}/diagnose_pca9685.sh"
            ;;
        realsense|camera)
            "${SCRIPT_DIR}/camera" diagnose 2>/dev/null || \
                echo "Run: ${SCRIPT_DIR}/camera diagnose"
            ;;
        *)
            echo -e "${RED}Error: Diagnostics not available for '$component'${NC}"
            echo "Available: phat, pca9685, realsense"
            exit 1
            ;;
    esac
}

show_status() {
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}Hardware Status${NC}"
    echo -e "${BLUE}========================================${NC}"
    echo ""

    echo "I2C Buses:"
    ls -1 /dev/i2c-* 2>/dev/null || echo "  None found"
    if command -v i2cdetect &>/dev/null && [ -e /dev/i2c-7 ]; then
        echo "  I2C7 scan:"
        i2cdetect -y 7 2>/dev/null | sed 's/^/    /' || true
    fi
    echo ""

    echo "USB Devices:"
    lsusb 2>/dev/null | head -12 || echo "  None found"
    echo ""

    echo "Serial Devices:"
    ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "  None found"
    echo ""

    echo "GPIO Devices:"
    ls -l /dev/gpiochip* 2>/dev/null || echo "  None found"
    echo ""

    echo "Audio Sources:"
    if command -v pactl &>/dev/null; then
        pactl list sources short 2>/dev/null | head -5 || echo "  None found"
    else
        echo "  pactl not available"
    fi
    echo ""

    echo "extlinux overlays:"
    grep -i "FDT_OVERLAYS" /boot/extlinux/extlinux.conf 2>/dev/null || echo "  None configured"
    echo ""

    echo "Groups ($USER):"
    groups "$USER"
}

list_components() {
    echo -e "${BLUE}Available Hardware Components:${NC}"
    echo ""
    echo "  realsense  - Intel RealSense cameras (D435/D455)"
    echo "  microphone - USB microphone"
    echo "  odrive     - ODrive motor controller"
    echo "  irobot     - iRobot Create/Roomba"
    echo "  phat       - SparkFun Auto pHAT (PCA9685 servo + ICM-20948 IMU)"
    echo ""
}

# ─────────────────────────────────────────────────────────────────────────────
# full-setup — single entry point
# ─────────────────────────────────────────────────────────────────────────────

full_setup() {
    echo ""
    echo -e "${CYAN}╔══════════════════════════════════════════╗${NC}"
    echo -e "${CYAN}║     Robot Hardware Full Setup            ║${NC}"
    echo -e "${CYAN}║     Jetson Orin Nano — first-time init   ║${NC}"
    echo -e "${CYAN}╚══════════════════════════════════════════╝${NC}"
    echo ""
    info "Project root: $PROJECT_ROOT"
    echo ""

    # ── Pass detection ──────────────────────────────────────────────────────
    # The I2C overlay requires a reboot to take effect. We detect which pass
    # we're on by checking whether /boot/extlinux/extlinux.conf already
    # contains the overlay line AND whether the 100kHz speed is active.

    local i2c_overlay_staged=false
    local i2c_overlay_active=false

    if grep -q "i2c7-100khz.dtbo" /boot/extlinux/extlinux.conf 2>/dev/null; then
        i2c_overlay_staged=true
    fi

    # After reboot with overlay active, the reported clock-frequency in
    # /proc/device-tree will be 0x000186a0 (100000 in big-endian hex).
    if [ -f /proc/device-tree/bus@0/i2c@c250000/clock-frequency ]; then
        local freq
        freq=$(xxd -e -g4 /proc/device-tree/bus@0/i2c@c250000/clock-frequency 2>/dev/null | awk '{print $2}' | head -1)
        if [ "$freq" = "000186a0" ]; then
            i2c_overlay_active=true
        fi
    fi

    if [ "$i2c_overlay_staged" = true ] && [ "$i2c_overlay_active" = false ]; then
        echo -e "${YELLOW}┌─────────────────────────────────────────────┐${NC}"
        echo -e "${YELLOW}│  REBOOT PENDING — Run full-setup again      │${NC}"
        echo -e "${YELLOW}│  after rebooting to complete setup.         │${NC}"
        echo -e "${YELLOW}└─────────────────────────────────────────────┘${NC}"
        echo ""
        info "The I2C7 100kHz overlay has been staged in extlinux.conf."
        info "It will take effect after a reboot."
        info ""
        info "Reboot now:  sudo reboot"
        info "Then re-run: ./scripts/hardware/setup_hardware.sh full-setup"
        exit 0
    fi

    # ── Pass 1: groups + udev + I2C overlay ─────────────────────────────────
    if [ "$i2c_overlay_active" = false ]; then
        echo -e "${BLUE}Pass 1 of 2 — System configuration + I2C overlay${NC}"
        echo ""

        setup_system_groups
        install_udev_rules
        "${SCRIPT_DIR}/setup_i2c.sh"

        echo ""
        echo -e "${YELLOW}╔══════════════════════════════════════════╗${NC}"
        echo -e "${YELLOW}║  REBOOT REQUIRED                         ║${NC}"
        echo -e "${YELLOW}╠══════════════════════════════════════════╣${NC}"
        echo -e "${YELLOW}║  The I2C7 100kHz overlay has been staged.║${NC}"
        echo -e "${YELLOW}║  Reboot then re-run full-setup to finish.║${NC}"
        echo -e "${YELLOW}╚══════════════════════════════════════════╝${NC}"
        echo ""
        echo "  sudo reboot"
        echo ""
        echo "  After reboot:"
        echo "  ./scripts/hardware/setup_hardware.sh full-setup"
        exit 0
    fi

    # ── Pass 2: verify I2C + install all drivers ────────────────────────────
    echo -e "${BLUE}Pass 2 of 2 — Driver installation and verification${NC}"
    echo ""

    step "Verifying I2C7 running at 100kHz"
    if command -v i2cdetect &>/dev/null; then
        info "Scanning I2C bus 7 for Auto pHAT devices..."
        i2cdetect -y 7 2>/dev/null || true
        echo ""
        if i2cdetect -y 7 2>/dev/null | grep -q "40"; then
            ok "PCA9685 servo controller detected at 0x40 on I2C7"
        else
            warn "PCA9685 not detected at 0x40 — run diagnose_pca9685.sh for full diagnostics"
        fi
    else
        warn "i2c-tools not installed; run: ./scripts/hardware/setup_hardware.sh install phat"
    fi

    step "Installing camera drivers"
    install_component realsense

    step "Installing microphone support"
    install_component microphone

    step "Setting up GPIO"
    "${SCRIPT_DIR}/setup_gpio.sh"

    step "Final hardware verification"
    verify_hardware

    echo ""
    echo -e "${GREEN}╔══════════════════════════════════════════╗${NC}"
    echo -e "${GREEN}║  Hardware setup complete!                ║${NC}"
    echo -e "${GREEN}╚══════════════════════════════════════════╝${NC}"
    echo ""
    info "Next steps:"
    info "  1. Pull and start the robot container:"
    info "       docker compose pull && docker compose up -d"
    info "  2. Check hardware topics:"
    info "       ros2 topic list"
    info "  3. Run PCA9685 diagnostics if servo not working:"
    info "       ./scripts/hardware/diagnose_pca9685.sh"
}

# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

case "${1:-}" in
    full-setup)
        full_setup
        ;;
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
