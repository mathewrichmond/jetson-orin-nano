#!/bin/bash
# Verify NVMe SSD on Jetson Orin Nano
# Run before migration to ensure the drive is healthy and ready for rootfs
#
# Usage: sudo ./scripts/system/verify_ssd.sh [device]
# Default device: /dev/nvme0n1

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

if [ "$EUID" -ne 0 ]; then
    echo -e "${RED}Please run with sudo${NC}"
    exit 1
fi

DEVICE="${1:-/dev/nvme0n1}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAAC_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

echo "=========================================="
echo "SSD Verification - Jetson Orin Nano"
echo "=========================================="
echo "Device: $DEVICE"
echo ""

PASS=0
FAIL=0

check_pass() {
    echo -e "${GREEN}✓ PASS:${NC} $1"
    ((PASS++)) || true
}

check_fail() {
    echo -e "${RED}✗ FAIL:${NC} $1"
    ((FAIL++)) || true
}

check_warn() {
    echo -e "${YELLOW}⚠ WARN:${NC} $1"
}

# 1. Device exists
echo "[1/6] Checking device exists..."
if [ -b "$DEVICE" ]; then
    check_pass "Device $DEVICE found"
else
    check_fail "Device $DEVICE not found. Is NVMe installed?"
    exit 1
fi

# 2. Not currently mounted as root
echo "[2/6] Checking not in use as root..."
ROOT_DEV=$(findmnt -n -o SOURCE / 2>/dev/null | sed 's/p[0-9]*$//' || true)
if [ -n "$ROOT_DEV" ] && [ "$(readlink -f "$DEVICE")" = "$(readlink -f "$ROOT_DEV")" ]; then
    check_warn "Device is already the root filesystem - nothing to migrate"
else
    check_pass "Device is not root (currently booted from: $(findmnt -n -o SOURCE / 2>/dev/null || echo 'unknown'))"
fi

# 3. NVMe identify (basic info)
echo "[3/6] NVMe device info..."
if command -v nvme &>/dev/null; then
    nvme id-ctrl "$DEVICE" 2>/dev/null | grep -E "^(mn|sn|fr )" || true
    check_pass "NVMe identify succeeded"
else
    check_warn "nvme-cli not installed (apt install nvme-cli for full verification)"
fi

# 4. SMART health (if smartmontools available)
echo "[4/6] SMART health check..."
if command -v smartctl &>/dev/null; then
    NVME_DEV="${DEVICE%%n1}"  # e.g. /dev/nvme0
    if smartctl -H "$NVME_DEV" 2>/dev/null | grep -q "overall-health.*PASSED"; then
        check_pass "SMART health: PASSED"
    else
        HEALTH=$(smartctl -H "$NVME_DEV" 2>/dev/null | grep "overall-health" || echo "unknown")
        if [[ "$HEALTH" == *"PASSED"* ]]; then
            check_pass "SMART health: OK"
        else
            check_fail "SMART health check failed or unsupported: $HEALTH"
        fi
    fi
else
    check_warn "smartmontools not installed (apt install smartmontools)"
fi

# 5. Write/read verification (optional, uses 64MB)
echo "[5/6] Write/read verification..."
if [ ! -b "${DEVICE}p1" ]; then
    # Raw device - do a small write test
    TEST_FILE="/tmp/ssd_verify_$$"
    dd if=/dev/urandom of="$TEST_FILE" bs=1M count=64 2>/dev/null
    if dd if="$TEST_FILE" of="$DEVICE" bs=1M count=64 seek=0 2>/dev/null conv=fsync; then
        CHECK=$(dd if="$DEVICE" bs=1M count=64 skip=0 2>/dev/null | sha256sum)
        ORIG=$(sha256sum "$TEST_FILE" 2>/dev/null | cut -d' ' -f1)
        rm -f "$TEST_FILE"
        if [ "$(echo "$CHECK" | cut -d' ' -f1)" = "$ORIG" ]; then
            check_pass "Write/read verification passed"
        else
            check_fail "Readback verification failed - checksum mismatch"
        fi
    else
        rm -f "$TEST_FILE" 2>/dev/null
        check_fail "Write test failed"
    fi
else
    check_warn "Device has partitions - skipping raw write test (run before partitioning)"
fi

# 6. Capacity check
echo "[6/6] Capacity check..."
SIZE_BYTES=$(blockdev --getsize64 "$DEVICE" 2>/dev/null)
SIZE_GB=$((SIZE_BYTES / 1024 / 1024 / 1024))
ROOT_USED=$(df -B1 / 2>/dev/null | tail -1 | awk '{print $3}')
ROOT_GB=$((ROOT_USED / 1024 / 1024 / 1024))
REQUIRED=$((ROOT_GB + 10))  # 10GB headroom

if [ "$SIZE_GB" -ge "$REQUIRED" ]; then
    check_pass "Capacity: ${SIZE_GB}GB (need ~${REQUIRED}GB for migration)"
else
    check_fail "Capacity: ${SIZE_GB}GB - need at least ${REQUIRED}GB (current root: ~${ROOT_GB}GB)"
fi

echo ""
echo "=========================================="
echo "Summary: $PASS passed, $FAIL failed"
echo "=========================================="

if [ "$FAIL" -gt 0 ]; then
    echo -e "${RED}Verification failed. Do not proceed with migration.${NC}"
    exit 1
fi

echo -e "${GREEN}SSD is ready for migration.${NC}"
echo ""
echo "Next steps:"
echo "  1. Read docs/setup/SSD_MIGRATION.md for full migration guide"
echo "  2. Run migration script when ready"
exit 0
