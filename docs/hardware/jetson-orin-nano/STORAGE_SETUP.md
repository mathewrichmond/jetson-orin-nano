# Storage Setup: NVMe SSD and SD Card

Complete guide for installing and configuring storage on the Jetson Orin Nano Developer Kit.

## Overview

The Jetson Orin Nano supports two storage options:

1. **NVMe SSD** (M.2 2280) - **Required for production use**
2. **microSD Card** - Optional fallback for recovery/testing

**CRITICAL**: The NVMe SSD must be **physically installed before flashing** with SDK Manager. You cannot flash to an SSD that isn't present.

## NVMe SSD Installation

### Compatible SSDs

The Jetson Orin Nano supports M.2 NVMe SSDs with the following specifications:

| Specification | Requirement |
|---------------|-------------|
| **Form Factor** | M.2 2280 (22mm x 80mm) |
| **Key Type** | M Key |
| **Interface** | NVMe PCIe Gen3 x4 or Gen4 x4 |
| **Capacity** | Minimum 128GB, **recommend 256GB or 512GB** |
| **Protocol** | NVMe 1.3 or later |

### Recommended SSDs

These SSDs are tested and known to work well:

- **Samsung 970 EVO Plus** - Excellent performance and reliability
- **WD Black SN750** - High endurance
- **Crucial P3** - Budget-friendly option
- **Samsung 980 PRO** - Premium Gen4 performance
- **Kingston NV2** - Budget option

**Avoid**:
- SATA M.2 SSDs (wrong protocol, won't work)
- M.2 2242/2260 form factors (too short, won't secure properly)
- Very cheap no-name brands (reliability issues)

### Physical Installation Procedure

**IMPORTANT**: Power must be completely disconnected before installing the SSD.

#### Required Tools

- Phillips #0 or #1 screwdriver
- M.2 mounting screw (included with Jetson kit)
- ESD wrist strap (recommended)

#### Installation Steps

1. **Power Off System**
   ```bash
   # If system is running
   sudo shutdown -h now
   
   # Wait for system to power down completely
   # Unplug DC power adapter
   ```

2. **Ground Yourself**
   - Touch a grounded metal object
   - Use ESD wrist strap if available
   - Avoid working on carpet

3. **Locate M.2 Slot**
   - Flip carrier board over to view underside
   - Identify M.2 slots:
     - **Longer slot** (near center): NVMe SSD - **USE THIS ONE**
     - Shorter slot: M.2 Key E (WiFi/BT) - not for storage

4. **Remove Mounting Screw**
   - Locate small Phillips screw at far end of NVMe slot
   - Unscrew and set aside (don't lose it!)

5. **Insert SSD**
   - Hold SSD by edges (avoid touching gold contacts)
   - Align notch on SSD with key in M.2 slot
   - Insert at 30-45 degree angle
   - Gently push SSD into connector until fully seated
   - SSD will be at an angle (this is normal)

6. **Secure SSD**
   - Press SSD down flat against carrier board
   - Hold down while threading screw back in
   - Tighten screw **gently** (do not overtighten!)
   - SSD should be flat and secure

7. **Verify Installation**
   - Gently tug on SSD edge - should not move
   - Check that gold contacts are fully inserted
   - Flip board back over

### Visual Reference

```
Top View (underside of carrier board):
                                  
  [M.2 Screw Hole]                
         |                        
         |                        
    [=========NVMe SSD=========]  ← Insert at angle, press flat
         |                        
    [M.2 Connector Slot]          
         |                        
  [SODIMM Module]                 
```

## microSD Card Setup

### When to Use SD Card

Use a microSD card for:
- **Emergency recovery** when NVMe won't boot
- **Temporary boot** for diagnostics and repair
- **Testing different JetPack versions** without reflashing NVMe
- **Initial setup** if you don't have an NVMe yet (not recommended for production)

### Recommended SD Cards

| Specification | Requirement |
|---------------|-------------|
| **Capacity** | Minimum 32GB, recommend 64GB+ |
| **Speed Class** | Class 10, UHS-I U3 or better |
| **Read Speed** | 90+ MB/s |
| **Write Speed** | 60+ MB/s |
| **Brand** | SanDisk, Samsung, Kingston (avoid no-name brands) |

**Recommended Cards**:
- SanDisk Extreme (64GB or 128GB)
- Samsung EVO Plus
- Kingston Canvas React

### Creating Bootable SD Card

#### Option 1: Using NVIDIA SD Card Image (Easiest)

1. **Download JetPack SD Card Image**
   ```bash
   # Visit NVIDIA Developer site
   # Download: Jetson Orin Nano SD Card Image (JetPack 5.1.3)
   # File: jetson-orin-nano-sd-card-image.zip
   ```

2. **Extract Image**
   ```bash
   unzip jetson-orin-nano-sd-card-image.zip
   # Produces: sd-blob.img or similar
   ```

3. **Flash to SD Card (Linux)**
   ```bash
   # Insert SD card, identify device
   lsblk
   # Look for your SD card (e.g., /dev/sdb, /dev/mmcblk0)
   
   # Flash image (CAREFUL: wrong device will destroy data!)
   sudo dd if=sd-blob.img of=/dev/sdX bs=4M status=progress
   sudo sync
   
   # Wait for completion (10-20 minutes for 32GB card)
   ```

4. **Flash to SD Card (macOS)**
   ```bash
   # Insert SD card, identify device
   diskutil list
   # Look for your SD card (e.g., /dev/disk4)
   
   # Unmount (don't eject!)
   diskutil unmountDisk /dev/diskN
   
   # Flash image
   sudo dd if=sd-blob.img of=/dev/rdiskN bs=4m
   sudo sync
   ```

5. **Flash to SD Card (Windows)**
   - Use **Balena Etcher** or **Rufus**
   - Select downloaded image file
   - Select SD card
   - Click "Flash"

#### Option 2: Using SDK Manager with SD Card Target

During SDK Manager flashing, select "SD Card" as target storage instead of "NVMe".

### Booting from SD Card

1. **Insert SD Card**
   - Power off Jetson
   - Insert microSD card into slot (push until click)
   - Located on underside near M.2 slots

2. **Power On**
   - Connect power adapter
   - System will automatically boot from SD card if no NVMe bootloader exists
   - If NVMe is bootable, it takes priority over SD card

3. **Force SD Card Boot** (if NVMe is also present)
   - Interrupt U-Boot during boot (requires serial console)
   - Press key when you see: "Hit any key to stop autoboot"
   - At U-Boot prompt:
     ```
     => setenv boot_targets mmc1 nvme0
     => boot
     ```

## Storage Configuration in SDK Manager

When flashing with SDK Manager, you'll be asked to select target storage:

### Storage Selection Screen

```
Target Hardware Configuration
├── Storage Device
│   ├── ● NVMe (recommended)
│   ├── ○ SD Card
│   └── ○ USB Drive (not recommended)
└── [Continue]
```

**Select**:
- **NVMe** - For production systems (requires SSD installed)
- **SD Card** - Only for testing or if no NVMe available

### Partition Layout

After flashing, storage is partitioned as follows:

#### NVMe Layout
```
/dev/nvme0n1
├── nvme0n1p1  (APP)     - Root filesystem (/)
├── nvme0n1p2  (TBC)     - Bootloader
├── nvme0n1p3  (RP1)     - Reserved
└── ... (additional partitions)
```

#### SD Card Layout
```
/dev/mmcblk0
├── mmcblk0p1  (APP)     - Root filesystem (/)
├── mmcblk0p2  (TBC)     - Bootloader
└── ... (additional partitions)
```

## Post-Installation Verification

After installing storage and flashing, verify everything is working:

### Check Root Filesystem Location

```bash
# Check what device is mounted as root
df -h /

# Expected output (NVMe):
# Filesystem      Size  Used Avail Use% Mounted on
# /dev/nvme0n1p1  233G  8.2G  213G   4% /

# Expected output (SD card):
# /dev/mmcblk0p1   29G  7.8G   20G  29% /
```

### Check NVMe Health

```bash
# Install nvme-cli if not present
sudo apt-get update
sudo apt-get install -y nvme-cli

# Check NVMe SMART data
sudo nvme smart-log /dev/nvme0n1

# Key metrics:
# - Temperature: Should be < 70°C
# - Available Spare: Should be 100%
# - Percentage Used: Should be low
```

### Check Storage Speed

```bash
# Test read speed
sudo hdparm -t /dev/nvme0n1

# Expected: 2000-3500 MB/sec for NVMe Gen3

# Test write speed
dd if=/dev/zero of=~/testfile bs=1M count=1024 conv=fdatasync
rm ~/testfile

# Expected: 1000-2000 MB/sec for NVMe Gen3
```

### View All Block Devices

```bash
lsblk -o NAME,SIZE,TYPE,MOUNTPOINT,MODEL

# Expected output:
# NAME         SIZE TYPE MOUNTPOINT MODEL
# mmcblk0     29.7G disk            
# └─mmcblk0p1 29.5G part            
# nvme0n1      477G disk            Samsung SSD 970 EVO Plus 500GB
# ├─nvme0n1p1  233G part /
# ├─nvme0n1p2   64M part
# └─...
```

## Troubleshooting

### SSD Not Detected

**Symptoms**:
- SDK Manager doesn't see NVMe during flash
- `lsblk` doesn't show `nvme0n1`

**Solutions**:
1. **Verify physical installation**
   - Power off, reseat SSD
   - Check screw is tight
   - Ensure gold contacts fully inserted

2. **Check SSD compatibility**
   - Must be NVMe (not SATA M.2)
   - Must be M.2 2280 form factor
   - Try a different SSD if available

3. **Check in UEFI/U-Boot**
   - Connect serial console
   - During boot, check for NVMe detection messages
   - Should see: "Scanning nvme 0:1..."

### SD Card Not Booting

**Symptoms**:
- Jetson doesn't boot from SD card
- Black screen or boot loop

**Solutions**:
1. **Verify card is bootable**
   - Re-flash image with dd or Etcher
   - Use a different SD card (some cards are incompatible)

2. **Check boot order**
   - NVMe takes priority if bootable
   - Remove NVMe or interrupt U-Boot to force SD boot

3. **Check SD card detection**
   ```bash
   # From a working boot (NVMe or serial console)
   ls /dev/mmcblk*
   # Should show: /dev/mmcblk0 and partitions
   ```

### Slow Performance

**NVMe feels slow**:
```bash
# Check if drive is in low power mode
sudo nvme get-feature /dev/nvme0n1 -f 0x0c

# Check for thermal throttling
sudo nvme smart-log /dev/nvme0n1 | grep -i temp

# If temperature > 70°C, add thermal pad or heatsink
```

**SD card feels slow**:
- This is normal - SD cards are 10-50x slower than NVMe
- Use NVMe for production systems

## Best Practices

### For Production Systems

- ✅ **Always use NVMe SSD** (not SD card)
- ✅ Use reputable SSD brands (Samsung, WD, Crucial)
- ✅ 256GB or larger capacity for logs and data
- ✅ Keep SD card as emergency fallback
- ✅ Monitor SSD health monthly
- ✅ Keep backups of important data

### For Development

- ✅ Keep bootable SD card with known-good JetPack image
- ✅ Test NVMe before production deployment
- ✅ Document your storage configuration
- ✅ Use serial console for storage troubleshooting

## Related Documentation

- [FLASHING.md](FLASHING.md) - SDK Manager flashing procedure
- [SERIAL_CONSOLE.md](SERIAL_CONSOLE.md) - Debug console for storage troubleshooting
- [RECOVERY.md](../../RECOVERY.md) - System recovery procedures

## External References

- **NVIDIA Storage Guide**: https://docs.nvidia.com/jetson/archives/r35.5.0/DeveloperGuide/text/SD/Bootloader.html
- **Supported Storage Devices**: Check NVIDIA Developer Forums for latest compatibility list
