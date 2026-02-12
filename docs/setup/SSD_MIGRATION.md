# SSD Migration & Docker-First Setup

Migrate from bare-metal SD card to Docker-first development on NVMe SSD while keeping the SD card as a fallback.

## Overview

| Phase | Description |
|-------|-------------|
| **1. Verify** | Confirm NVMe SSD is healthy and ready |
| **2. Migrate** | Clone rootfs to SSD, configure dual-boot |
| **3. Docker-First** | Configure Docker storage and development workflow on SSD |
| **4. Fallback** | SD card remains bootable for recovery |

## Prerequisites

- Jetson Orin Nano with NVMe M.2 slot (PCIe Gen3)
- NVMe SSD installed (SPCC M.2 PCIe SSD or similar)
- Currently booted from SD card with working system
- **Backup** important data before migration

---

## Phase 1: Verify the SSD

Run the verification script to ensure the drive is healthy:

```bash
cd /home/nano/src/jetson-orin-nano
sudo ./scripts/system/verify_ssd.sh
```

Or with a specific device:

```bash
sudo ./scripts/system/verify_ssd.sh /dev/nvme0n1
```

The script checks:
- Device presence and NVMe identify
- SMART health (if smartmontools installed)
- Write/read verification (64MB test)
- Capacity vs. current root usage

**Optional**: Install full verification tools:
```bash
sudo apt install -y nvme-cli smartmontools
```

---

## Phase 2: Migration Options

### Migration Decision: Flash vs In-Place

| Factor | In-Place Migration | Host PC Flashing |
|--------|--------------------|------------------|
| **Host PC** | Not required | Requires Ubuntu 22.04 host |
| **Recovery mode** | Not required | Jumper + USB to host |
| **Result** | Clone of current system | Clean Jetson Linux image |
| **Customizations** | Preserved (configs, packages, users) | Lost – must reapply |
| **Time** | ~30 min (rsync) | ~45 min (flash + reinstall) |
| **Best for** | Existing working setup you want to keep | Fresh start, minimal base, or corrupted system |

**Recommendation**:
- **In-place** if your SD system is healthy and you want to keep all configs, udev rules, SSH keys, Docker setup, and user data. SD remains as fallback.
- **Flash** if you want a minimal clean base (e.g., for a new production Docker host) or if the current system is unreliable.

For a **light Docker host** that will run containerized microservices, either works:
- In-place: Clone current system, then trim and containerize over time.
- Flash: Start minimal, add only Docker + NVIDIA runtime, then deploy containers.

See [docs/setup/PRODUCTION_SYSTEM_DESIGN.md](PRODUCTION_SYSTEM_DESIGN.md) for production setup (synced mounts, headless, microservices).

---

### Option A: In-Place Migration (Recommended)

Clone the running system to NVMe while keeping the SD card as fallback. No host PC required.

1. **Run migration script** (when available):
   ```bash
   sudo ./scripts/system/migrate_to_ssd.sh
   ```

2. **Or follow manual steps** below.

#### Manual Migration Steps

**Step 1: Partition the NVMe**
```bash
sudo parted /dev/nvme0n1 --script mklabel gpt
sudo parted /dev/nvme0n1 --script mkpart primary ext4 1MiB 100%
sudo parted /dev/nvme0n1 --script set 1 esp off
sudo mkfs.ext4 -L nvme-root /dev/nvme0n1p1
```

**Step 2: Clone rootfs**
```bash
sudo mkdir -p /mnt/nvme_root
sudo mount /dev/nvme0n1p1 /mnt/nvme_root
sudo rsync -avxHAX --info=progress2 \
  --exclude=/mnt/nvme_root \
  --exclude=/proc \
  --exclude=/sys \
  --exclude=/dev \
  --exclude=/run \
  --exclude=/tmp \
  --exclude=/lost+found \
  --exclude=/boot/efi \
  / /mnt/nvme_root/
```

**Step 3: Get NVMe root PARTUUID**
```bash
ls -l /dev/disk/by-partuuid/ | grep nvme0n1p1
# Note the PARTUUID (e.g., a1b2c3d4-e5f6-7890-abcd-ef1234567890)
```

**Step 4: Update extlinux for dual-boot**

Edit `/boot/extlinux/extlinux.conf`:

```bash
sudo nano /boot/extlinux/extlinux.conf
```

- Change `DEFAULT primary` to `DEFAULT nvme` (NVMe as default)
- Add new label **before** the closing of the file:

```
LABEL nvme
      MENU LABEL Boot from NVMe (SSD) - default
      LINUX /boot/Image
      INITRD /boot/initrd
      APPEND ${cbootargs} root=PARTUUID=<NVME_PARTUUID> rw rootwait rootfstype=ext4 mminit_loglevel=4 console=ttyTCU0,115200 firmware_class.path=/etc/firmware fbcon=map:0 video=efifb:off console=tty0 nv-auto-config

LABEL sd_fallback
      MENU LABEL Boot from SD card (fallback)
      LINUX /boot/Image
      INITRD /boot/initrd
      APPEND ${cbootargs} root=PARTUUID=7ab45be1-5de8-4e5f-8859-05b01bdcf36d rw rootwait rootfstype=ext4 mminit_loglevel=4 console=ttyTCU0,115200 firmware_class.path=/etc/firmware fbcon=map:0 video=efifb:off console=tty0 nv-auto-config
```

Replace `<NVME_PARTUUID>` with the actual UUID from Step 3.

**Step 5: Update fstab on NVMe** (so NVMe root mounts correctly)

```bash
# Check current root UUID
sudo blkid /dev/nvme0n1p1

# Edit fstab on NVMe root - ensure root uses PARTUUID or UUID
sudo nano /mnt/nvme_root/etc/fstab
# Update the root (/) line to use the NVMe partition UUID
```

**Step 6: Sync and reboot**

```bash
sync
sudo umount /mnt/nvme_root
sudo reboot
```

On boot, the extlinux menu (30 second timeout) lets you choose:
- **NVMe** (default) – SSD system
- **SD fallback** – Original SD card system

---

### Option B: Host PC Flashing (Clean Install)

If you have an Ubuntu 22.04 host PC and want a clean image on NVMe:

1. Download Jetson Linux from [NVIDIA Developer](https://developer.nvidia.com/embedded/jetson-linux-archive)
2. Put Jetson in **Recovery Mode**: J14 pins 9–10 jumpered, power on
3. Connect USB to host, run:
   ```bash
   sudo ./tools/kernel_flash/l4t_initrd_flash.sh --external-device nvme0n1p1 \
     -c tools/kernel_flash/flash_l4t_external.xml \
     -p"-c bootloader/generic/cfg/flash_t234_qspi.xml" \
     --showlogs --network usb0 jetson-orin-nano-devkit internal
   ```

This flashes a fresh image to NVMe. The SD card remains usable if you later flash it again for fallback.

---

## Phase 3: Docker-First Setup on SSD

After booting from NVMe:

### 1. Configure Docker data root on SSD

Docker default data is in `/var/lib/docker`. On SSD root this is fine. For performance, ensure:

```bash
# Verify Docker uses SSD (root is on NVMe)
df -h /var/lib/docker
# Should show your NVMe mount
```

### 2. Install NVIDIA Container Toolkit (for GPU in Docker)

```bash
# If not already installed
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
```

### 3. Development workflow

Use the existing `Dockerfile` and `Dockerfile.ci` from the repo:

```bash
# Build for development
docker build -t isaac-robot:dev .

# Or use make targets
make docker-build
make docker-test
```

### 4. Data persistence

- **Project code**: Bind-mount from host (e.g. `-v /home/nano/src/jetson-orin-nano:/workspace`)
- **Docker volumes**: Stored on SSD under `/var/lib/docker`
- **Session data** (`/home/nano/data`): Lives on SSD root

---

## Phase 4: SD Card Fallback

### Booting from SD

1. Power on the Jetson
2. During the 30-second extlinux countdown, press a key
3. Select **"Boot from SD card (fallback)"**
4. Boot continues from the original SD system

### Keeping SD up to date

After major changes on NVMe (e.g. kernel updates), you can optionally sync back to SD:

```bash
# Boot from NVMe, then sync selected dirs to SD (mounted at /mnt/sd_fallback)
# Only if you want SD to mirror critical updates
sudo mount /dev/mmcblk0p1 /mnt/sd_fallback
sudo rsync -av /boot /mnt/sd_fallback/
sync
sudo umount /mnt/sd_fallback
```

This keeps the SD bootable for recovery.

---

## Troubleshooting

### NVMe not detected at boot

- Ensure the drive is firmly seated
- Try a PCIe Gen3 (not Gen4) SSD if using Gen4
- Check carrier board compatibility

### Boot fails with root=PARTUUID=...

- Verify PARTUUID: `ls -l /dev/disk/by-partuuid/`
- Ensure initrd has NVMe modules (standard L4T should include them)
- Use SD fallback to fix `extlinux.conf`

### Docker fails with GPU

```bash
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

### Full recovery

1. Boot from SD (fallback)
2. Edit `/boot/extlinux/extlinux.conf`: set `DEFAULT primary` (or `sd_fallback`)
3. Reboot

---

## Reference

- [NVIDIA Jetson Orin Nano Developer Kit](https://developer.nvidia.com/embedded/jetson-orin-nano-dev-kit)
- [Jetson Linux Download](https://developer.nvidia.com/embedded/jetson-linux-archive)
- Boot config: `/boot/extlinux/extlinux.conf`
- Current root PARTUUID (SD): `7ab45be1-5de8-4e5f-8859-05b01bdcf36d`
