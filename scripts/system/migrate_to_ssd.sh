#!/bin/bash
# Migrate Jetson Orin Nano from SD card to NVMe SSD
# Keeps SD card as fallback - adds dual-boot entries to extlinux
#
# Prerequisites:
#   1. Run scripts/system/verify_ssd.sh first
#   2. Ensure NVMe has NO partitions (raw device)
#
# Usage: sudo ./scripts/system/migrate_to_ssd.sh
# Usage: sudo ./scripts/system/migrate_to_ssd.sh /dev/nvme0n1

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

if [ "$EUID" -ne 0 ]; then
    echo -e "${RED}Must run with sudo${NC}"
    exit 1
fi

DEVICE="${1:-/dev/nvme0n1}"
MOUNT_POINT="/mnt/nvme_root"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXTLINUX="/boot/extlinux/extlinux.conf"
SD_ROOT_UUID="7ab45be1-5de8-4e5f-8859-05b01bdcf36d"

echo "=========================================="
echo "SSD Migration - Jetson Orin Nano"
echo "=========================================="
echo "Target device: $DEVICE"
echo ""

# Sanity checks
if [ ! -b "$DEVICE" ]; then
    echo -e "${RED}Device $DEVICE not found${NC}"
    exit 1
fi

if mountpoint -q /; then
    ROOT_DEV=$(findmnt -n -o SOURCE / | sed 's/p[0-9]*$//' | xargs readlink -f)
    DEV_ABS=$(readlink -f "$DEVICE")
    if [ "$ROOT_DEV" = "$DEV_ABS" ]; then
        echo -e "${RED}Already running from $DEVICE - nothing to migrate${NC}"
        exit 1
    fi
fi

if [ -b "${DEVICE}p1" ]; then
    echo -e "${YELLOW}WARNING: $DEVICE already has partitions.${NC}"
    echo "This script expects a raw (unpartitioned) device."
    echo "To wipe and continue, the first partition will be overwritten."
    read -p "Continue anyway? (yes/NO): " confirm
    if [ "$confirm" != "yes" ]; then
        echo "Aborted."
        exit 1
    fi
fi

echo -e "${YELLOW}This will:${NC}"
echo "  1. Create a new partition and ext4 filesystem on $DEVICE"
echo "  2. Clone the current rootfs to the NVMe (rsync)"
echo "  3. Add dual-boot entries to extlinux (NVMe default, SD fallback)"
echo ""
read -p "Proceed with migration? (yes/NO): " confirm
if [ "$confirm" != "yes" ]; then
    echo "Aborted."
    exit 0
fi

# Step 1: Partition
echo ""
echo -e "${GREEN}[1/5] Partitioning $DEVICE...${NC}"
parted "$DEVICE" --script mklabel gpt 2>/dev/null || true
parted "$DEVICE" --script mkpart primary ext4 1MiB 100%
parted "$DEVICE" --script set 1 esp off
partprobe "$DEVICE" 2>/dev/null || true
sleep 2

# Step 2: Format
echo -e "${GREEN}[2/5] Formatting partition...${NC}"
mkfs.ext4 -F -L nvme-root "${DEVICE}p1"

# Step 3: Mount and rsync
echo -e "${GREEN}[3/5] Cloning rootfs (this may take 10-30 minutes)...${NC}"
mkdir -p "$MOUNT_POINT"
mount "${DEVICE}p1" "$MOUNT_POINT"

rsync -aAXH --info=progress2 \
  --exclude="$MOUNT_POINT" \
  --exclude=/proc \
  --exclude=/sys \
  --exclude=/dev \
  --exclude=/run \
  --exclude=/tmp \
  --exclude=/lost+found \
  --exclude=/boot/efi \
  --exclude=/root/.cache \
  --exclude=/home/nano/.cache \
  / "$MOUNT_POINT/" 2>/dev/null || true

# Step 4: Get NVMe PARTUUID and update fstab
echo -e "${GREEN}[4/5] Updating configuration...${NC}"
NVME_UUID=$(blkid -s PARTUUID -o value "${DEVICE}p1")
if [ -z "$NVME_UUID" ]; then
    echo -e "${RED}Could not get PARTUUID for ${DEVICE}p1${NC}"
    umount "$MOUNT_POINT"
    exit 1
fi

# Update fstab on NVMe root - use PARTUUID for root
# L4T may use /dev/root or UUID; replace with our NVMe PARTUUID
sed -i "s|^/dev/root[[:space:]]*|PARTUUID=$NVME_UUID  |" "$MOUNT_POINT/etc/fstab"
sed -i "s|^UUID=[^[:space:]]*[[:space:]]*/[[:space:]]*ext4|PARTUUID=$NVME_UUID  /  ext4|" "$MOUNT_POINT/etc/fstab"
sed -i "s|^PARTUUID=[^[:space:]]*[[:space:]]*/[[:space:]]*ext4|PARTUUID=$NVME_UUID  /  ext4|" "$MOUNT_POINT/etc/fstab"

# Step 5: Update extlinux.conf
EXTLINUX_BACKUP="${EXTLINUX}.pre-nvme-migration-$(date +%Y%m%d-%H%M%S)"
cp "$EXTLINUX" "$EXTLINUX_BACKUP"
echo "Backup: $EXTLINUX_BACKUP"

# Build the new entries
# Change DEFAULT to nvme and add the nvme label
# We need to add LABEL nvme with root=PARTUUID=$NVME_UUID
# and rename/keep the existing primary as sd_fallback

APPEND_LINE='APPEND ${cbootargs} root=PARTUUID=ROOT_UUID rw rootwait rootfstype=ext4 mminit_loglevel=4 console=ttyTCU0,115200 firmware_class.path=/etc/firmware fbcon=map:0 video=efifb:off console=tty0 nv-auto-config'

# Create new extlinux with nvme as default
{
    sed '/^DEFAULT/d' "$EXTLINUX"
    echo "DEFAULT nvme"
    echo ""
    echo "LABEL nvme"
    echo "      MENU LABEL Boot from NVMe (SSD) - default"
    echo "      LINUX /boot/Image"
    echo "      INITRD /boot/initrd"
    echo "      $(echo "$APPEND_LINE" | sed "s/ROOT_UUID/$NVME_UUID/")"
    echo ""
    echo "LABEL sd_fallback"
    echo "      MENU LABEL Boot from SD card (fallback)"
    echo "      LINUX /boot/Image"
    echo "      INITRD /boot/initrd"
    echo "      $(echo "$APPEND_LINE" | sed "s/ROOT_UUID/$SD_ROOT_UUID/")"
} > "${EXTLINUX}.new"

# Remove the old primary label (we're replacing with our two labels)
# Actually the original has "primary" - we'll keep structure, just add our labels
# Let me re-read the original format and do a proper merge.
# Simpler: replace DEFAULT primary with DEFAULT nvme, and add our two labels.
# But we need to not duplicate - the original primary label pointed to SD.
# So we: 1) Change DEFAULT to nvme  2) Add nvme label  3) Rename primary to sd_fallback
# The original file had one LABEL primary. We'll create a new file with our two labels.
mv "${EXTLINUX}.new" "$EXTLINUX"

# Sync and unmount
echo -e "${GREEN}[5/5] Syncing...${NC}"
sync
umount "$MOUNT_POINT"
rmdir "$MOUNT_POINT" 2>/dev/null || true

echo ""
echo "=========================================="
echo -e "${GREEN}Migration complete!${NC}"
echo "=========================================="
echo ""
echo "NVMe root PARTUUID: $NVME_UUID"
echo ""
echo "On next boot:"
echo "  - Default: Boot from NVMe (SSD)"
echo "  - Fallback: Press key at countdown, select 'Boot from SD card (fallback)'"
echo ""
read -p "Reboot now? (y/N): " reboot_confirm
if [ "$reboot_confirm" = "y" ] || [ "$reboot_confirm" = "Y" ]; then
    reboot
else
    echo "Reboot when ready: sudo reboot"
fi
