#!/bin/bash
# I2C Hardware Setup — Jetson Orin Nano
#
# Configures I2C bus 7 (GPIO header) for the SparkFun Auto pHAT:
#   - Installs i2c-tools
#   - Adds user to i2c group
#   - Installs udev rules for container access to /dev/i2c-*
#   - Applies the pre-compiled 100kHz device tree overlay via extlinux.conf
#
# SAFE APPROACH: Uses a device tree overlay (.dtbo) — NOT direct DTB editing.
# The overlay is reversible: remove the FDT_OVERLAYS line from extlinux.conf
# to revert. The main kernel DTB is never modified.
#
# Run as part of:  ./scripts/hardware/setup_hardware.sh full-setup
# Run standalone:  sudo ./scripts/hardware/setup_i2c.sh
#
# A REBOOT IS REQUIRED after this script for the clock-frequency change
# to take effect. setup_hardware.sh full-setup handles this automatically.

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
echo -e "${BLUE}=== I2C Bus 7 Setup (GPIO header / Auto pHAT) ===${NC}"
echo ""

# ── 1. i2c-tools ─────────────────────────────────────────────────────────────
echo "[1/5] Installing i2c-tools..."
if ! command -v i2cdetect &>/dev/null; then
    sudo apt-get install -y -qq i2c-tools
    ok "i2c-tools installed"
else
    ok "i2c-tools already installed ($(i2cdetect --version 2>&1 | head -1))"
fi

# ── 2. i2c group membership ───────────────────────────────────────────────────
echo ""
echo "[2/5] Checking i2c group membership..."
if ! getent group i2c &>/dev/null; then
    sudo groupadd i2c
    ok "Created i2c group"
fi

if ! groups "$USER" | grep -qw "i2c"; then
    sudo usermod -aG i2c "$USER"
    ok "Added $USER to i2c group"
else
    ok "$USER already in i2c group"
fi

# ── 3. Verify I2C bus 7 exists ───────────────────────────────────────────────
echo ""
echo "[3/5] Checking I2C bus availability..."
if [ -e /dev/i2c-7 ]; then
    ok "I2C bus 7 exists: /dev/i2c-7"
else
    warn "I2C bus 7 not found — available buses:"
    ls -1 /dev/i2c-* 2>/dev/null | sed 's/^/    /' || echo "    (none)"
    warn "Bus 7 should appear after first boot. Continuing overlay setup."
fi

# ── 4. Verify DT node path ───────────────────────────────────────────────────
echo ""
echo "[4/5] Verifying device tree node path for I2C7..."

DT_ALIAS_FILE="/proc/device-tree/aliases/i2c7"
# JetPack 6.x: /bus@0/i2c@c250000; older: /i2c@31e0000
EXPECTED_NODES="i2c@c250000 i2c@31e0000"
DTBO_SRC="${PROJECT_ROOT}/config/hardware/device_tree/i2c7-100khz.dtbo"
DTBO_DST="/boot/dtb/i2c7-100khz.dtbo"
EXTLINUX="/boot/extlinux/extlinux.conf"

if [ -f "$DT_ALIAS_FILE" ]; then
    # The alias value is a null-terminated string like "/bus@0/i2c@c250000"
    ACTUAL_NODE=$(cat "$DT_ALIAS_FILE" | tr -d '\0')
    info "I2C7 alias resolves to: $ACTUAL_NODE"

    MATCHED=false
    for node in $EXPECTED_NODES; do
        if echo "$ACTUAL_NODE" | grep -q "$node"; then
            MATCHED=true
            break
        fi
    done
    if $MATCHED; then
        ok "DT node path confirmed: $ACTUAL_NODE"
    else
        echo ""
        fail "Unexpected I2C7 node path: $ACTUAL_NODE (expected one of: $EXPECTED_NODES)"
        echo "  The overlay in config/hardware/device_tree/i2c7-100khz.dts"
        echo "  targets a known path which does not match this JetPack version."
        echo ""
        echo "  To fix:"
        echo "    1. Edit config/hardware/device_tree/i2c7-100khz.dts"
        echo "       Change target-path to: \"$ACTUAL_NODE\""
        echo "    2. Recompile on WSL2:"
        echo "       dtc -O dtb -o config/hardware/device_tree/i2c7-100khz.dtbo \\"
        echo "           -b 0 -@ config/hardware/device_tree/i2c7-100khz.dts"
        echo "    3. Commit and re-run this script"
        exit 1
    fi
else
    warn "/proc/device-tree/aliases/i2c7 not found — skipping node path check"
    warn "This is normal on first boot before the device tree is fully loaded."
fi

# ── 5. Install overlay and patch extlinux.conf ───────────────────────────────
echo ""
echo "[5/5] Installing I2C7 100kHz device tree overlay..."

if [ ! -f "$DTBO_SRC" ]; then
    fail "Pre-compiled overlay not found: $DTBO_SRC"
fi

# Copy .dtbo to /boot/dtb/
if [ ! -f "$DTBO_DST" ] || ! diff -q "$DTBO_SRC" "$DTBO_DST" &>/dev/null; then
    sudo cp "$DTBO_SRC" "$DTBO_DST"
    ok "Overlay installed: $DTBO_DST"
else
    ok "Overlay already up to date: $DTBO_DST"
fi

# Patch extlinux.conf (idempotent)
if [ ! -f "$EXTLINUX" ]; then
    fail "extlinux.conf not found at $EXTLINUX"
fi

if grep -q "i2c7-100khz.dtbo" "$EXTLINUX"; then
    ok "extlinux.conf already references i2c7-100khz.dtbo"
else
    # Back up extlinux.conf before modifying
    sudo cp "$EXTLINUX" "${EXTLINUX}.bak.$(date +%Y%m%d_%H%M%S)"
    ok "Backed up extlinux.conf"

    # Insert FDT_OVERLAYS line after the LABEL line (or append to the default entry)
    # JetPack 6 extlinux.conf uses FDT_OVERLAYS on its own line
    if grep -q "^      FDT_OVERLAYS" "$EXTLINUX"; then
        # Append to existing FDT_OVERLAYS line
        sudo sed -i 's|^\(      FDT_OVERLAYS .*\)|\1 /boot/dtb/i2c7-100khz.dtbo|' "$EXTLINUX"
        ok "Appended to existing FDT_OVERLAYS entry in extlinux.conf"
    else
        # Insert FDT_OVERLAYS before the APPEND line in the default boot entry
        sudo sed -i '/^      APPEND/i\      FDT_OVERLAYS /boot/dtb/i2c7-100khz.dtbo' "$EXTLINUX"
        ok "Added FDT_OVERLAYS to extlinux.conf"
    fi

    info "extlinux.conf updated:"
    grep -E "FDT_OVERLAYS|LABEL|LINUX" "$EXTLINUX" | head -10 | sed 's/^/    /'
fi

echo ""
echo -e "${GREEN}=== I2C setup complete ===${NC}"
echo ""
info "Changes made:"
info "  - i2c-tools installed"
info "  - $USER added to i2c group"
info "  - 100kHz overlay installed to $DTBO_DST"
info "  - extlinux.conf updated with FDT_OVERLAYS"
echo ""
echo -e "${YELLOW}  *** REBOOT REQUIRED for I2C frequency change to take effect ***${NC}"
echo ""
info "After reboot, verify with:"
info "  sudo i2cdetect -y 7          # should show 40 (PCA9685)"
info "  xxd -e -g4 /proc/device-tree/bus@0/i2c@c250000/clock-frequency"
info "  # should output: 000186a0 (100000 decimal)"
echo ""
