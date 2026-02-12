#!/bin/bash
# Add SD card fallback to bootloader for headless recovery
#
# Run with: sudo ./scripts/system/add_sd_fallback.sh

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

if [ "$EUID" -ne 0 ]; then
    echo -e "${RED}Must run with sudo${NC}"
    exit 1
fi

EXTLINUX="/boot/extlinux/extlinux.conf"
NVME_UUID="1cbf86a1-af7d-4aeb-915b-c7a4d7f7e9fa"
SD_UUID="7ab45be1-5de8-4e5f-8859-05b01bdcf36d"

echo "=========================================="
echo "Add SD Fallback to Bootloader"
echo "=========================================="
echo ""

# Backup current config
BACKUP="${EXTLINUX}.backup-$(date +%Y%m%d-%H%M%S)"
cp "$EXTLINUX" "$BACKUP"
echo "Backup created: $BACKUP"

# Check if already has nvme/sd_fallback labels
if grep -q "LABEL nvme" "$EXTLINUX" && grep -q "LABEL sd_fallback" "$EXTLINUX"; then
    echo -e "${YELLOW}Dual-boot already configured${NC}"
    exit 0
fi

# Create new extlinux with dual-boot
APPEND_LINE='APPEND ${cbootargs} root=PARTUUID=ROOT_UUID rw rootwait rootfstype=ext4 mminit_loglevel=4 console=ttyTCU0,115200 firmware_class.path=/etc/firmware fbcon=map:0 video=efifb:off console=tty0 nv-auto-config'

cat > "$EXTLINUX" <<EOF
TIMEOUT 30
DEFAULT nvme

MENU TITLE Isaac Robot Boot Options

LABEL nvme
      MENU LABEL Boot from NVMe (SSD) - Production
      LINUX /boot/Image
      INITRD /boot/initrd
      $(echo "$APPEND_LINE" | sed "s/ROOT_UUID/$NVME_UUID/")

LABEL sd_fallback
      MENU LABEL Boot from SD card (Recovery)
      LINUX /boot/Image
      INITRD /boot/initrd
      $(echo "$APPEND_LINE" | sed "s/ROOT_UUID/$SD_UUID/")
EOF

echo ""
echo -e "${GREEN}Dual-boot configured!${NC}"
echo ""
echo "Boot options:"
echo "  - Default: NVMe (production system)"
echo "  - Fallback: SD card (original system)"
echo "  - Timeout: 30 seconds"
echo ""
echo "To test SD fallback:"
echo "  1. Reboot: sudo reboot"
echo "  2. Interrupt boot (press key during countdown)"
echo "  3. Select 'Boot from SD card (Recovery)'"
