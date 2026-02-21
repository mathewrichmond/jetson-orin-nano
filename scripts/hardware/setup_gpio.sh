#!/bin/bash
# GPIO Setup — Jetson Orin Nano
#
# Configures host-level GPIO access for the Jetson GPIO header:
#   - Adds user to gpio group
#   - Installs Jetson.GPIO Python library
#   - Installs udev rules for /dev/gpiochip* container access
#   - Verifies gpiochip devices are accessible
#
# Run as part of:  ./scripts/hardware/setup_hardware.sh full-setup
# Run standalone:  sudo ./scripts/hardware/setup_gpio.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m'

ok()   { echo -e "  ${GREEN}[OK]${NC} $*"; }
warn() { echo -e "  ${YELLOW}[WARN]${NC} $*"; }
info() { echo -e "  ${BLUE}$*${NC}"; }
fail() { echo -e "  ${RED}[FAIL]${NC} $*"; exit 1; }

echo ""
echo -e "${BLUE}=== GPIO Setup (Jetson 40-pin header) ===${NC}"
echo ""

# ── 1. gpio group ─────────────────────────────────────────────────────────────
echo "[1/4] Configuring gpio group..."
if ! getent group gpio &>/dev/null; then
    sudo groupadd gpio
    ok "Created gpio group"
else
    ok "gpio group already exists"
fi

if ! groups "$USER" | grep -qw "gpio"; then
    sudo usermod -aG gpio "$USER"
    ok "Added $USER to gpio group"
else
    ok "$USER already in gpio group"
fi

# ── 2. Jetson.GPIO Python library ─────────────────────────────────────────────
echo ""
echo "[2/4] Installing Jetson.GPIO Python library..."

if python3 -c "import Jetson.GPIO" 2>/dev/null; then
    INSTALLED_VER=$(python3 -c "import Jetson.GPIO; print(Jetson.GPIO.VERSION)" 2>/dev/null || echo "unknown")
    ok "Jetson.GPIO already installed (version: $INSTALLED_VER)"
else
    # JetPack ships python3-jetson-gpio via apt on L4T; prefer that over pip
    if apt-cache show python3-jetson-gpio &>/dev/null 2>&1; then
        sudo apt-get install -y -qq python3-jetson-gpio
        ok "Installed python3-jetson-gpio via apt"
    else
        # Fall back to pip (works on both native and Docker builds)
        pip3 install --user Jetson.GPIO
        ok "Installed Jetson.GPIO via pip"
    fi
fi

# ── 3. gpiochip udev permissions ─────────────────────────────────────────────
echo ""
echo "[3/4] Checking gpiochip udev permissions..."

# The consolidated rules file covers gpio — check it's installed
if [ -f /etc/udev/rules.d/99-robot-hardware.rules ]; then
    if grep -q "gpiochip" /etc/udev/rules.d/99-robot-hardware.rules; then
        ok "GPIO udev rules already installed (99-robot-hardware.rules)"
    else
        warn "99-robot-hardware.rules exists but missing gpiochip entry"
        warn "Re-run: ./scripts/hardware/setup_hardware.sh full-setup"
    fi
else
    # Install a minimal gpio-only rule as fallback
    sudo tee /etc/udev/rules.d/99-gpio.rules > /dev/null << 'EOF'
# Minimal GPIO rules (full rules in 99-robot-hardware.rules)
SUBSYSTEM=="gpio", KERNEL=="gpiochip*", GROUP="gpio", MODE="0660"
SUBSYSTEM=="gpio", GROUP="gpio", MODE="0660"
EOF
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    ok "Installed minimal GPIO udev rules (99-gpio.rules)"
fi

# ── 4. Verify gpiochip devices ───────────────────────────────────────────────
echo ""
echo "[4/4] Verifying GPIO devices..."

if ls /dev/gpiochip* &>/dev/null 2>&1; then
    CHIP_COUNT=$(ls /dev/gpiochip* 2>/dev/null | wc -l)
    ok "Found $CHIP_COUNT gpiochip device(s):"
    ls -l /dev/gpiochip* 2>/dev/null | sed 's/^/    /'
    echo ""

    # Quick permission check — can we open the chip?
    CHIP=$(ls /dev/gpiochip* 2>/dev/null | head -1)
    if python3 - "$CHIP" << 'PYEOF' 2>/dev/null
import sys
import os
try:
    fd = os.open(sys.argv[1], os.O_RDONLY)
    os.close(fd)
    print(f"  Access check: {sys.argv[1]} readable")
    sys.exit(0)
except PermissionError:
    print(f"  Access check: {sys.argv[1]} permission denied (may need re-login)")
    sys.exit(1)
PYEOF
    then
        ok "GPIO device accessible"
    else
        warn "GPIO device not readable by current user"
        warn "Log out and back in for group changes to take effect, or run: newgrp gpio"
    fi
else
    warn "No /dev/gpiochip* devices found"
    info "Jetson GPIO chips appear after boot — verify after reboot"
fi

echo ""
echo -e "${GREEN}=== GPIO setup complete ===${NC}"
echo ""
info "Jetson 40-pin header GPIO:"
info "  Library:    import Jetson.GPIO as GPIO"
info "  Chip mode:  GPIO.setmode(GPIO.BOARD)"
info "  Docs:       https://github.com/NVIDIA/jetson-gpio"
echo ""
info "For the Auto pHAT motor pins, see:"
info "  config/hardware/phat_params.yaml (motor_*_pin parameters)"
echo ""
