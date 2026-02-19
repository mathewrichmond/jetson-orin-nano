# Flashing Guide: Recovery Mode and SDK Manager

Complete guide for flashing JetPack to the Jetson Orin Nano Developer Kit using NVIDIA SDK Manager.

## Overview

Flashing the Jetson Orin Nano requires:
1. **Ubuntu host PC** (x86-64, Ubuntu 20.04 or 22.04)
2. **NVMe SSD installed** in Jetson (see [STORAGE_SETUP.md](STORAGE_SETUP.md))
3. **Force Recovery Mode** enabled on Jetson
4. **USB-C connection** between Jetson and host PC
5. **NVIDIA SDK Manager** running on host PC

**Time Required**: 45-90 minutes (including downloads)

## Prerequisites

### Host PC Requirements

| Requirement | Specification |
|-------------|---------------|
| **Operating System** | Ubuntu 20.04 LTS or 22.04 LTS (x86-64) |
| **CPU** | x86-64 processor (Intel or AMD) |
| **RAM** | 8GB minimum, 16GB recommended |
| **Disk Space** | 40GB free space minimum (100GB recommended) |
| **Internet** | Stable high-speed connection |
| **USB Port** | USB 2.0 or 3.0 Type-A or Type-C |
| **Account** | NVIDIA Developer account (free) |

**CRITICAL**: SDK Manager only runs on Ubuntu x86-64. It does NOT work on:
- ❌ Windows (use Ubuntu VM or dual boot)
- ❌ macOS (use Ubuntu VM)
- ❌ ARM/ARM64 Linux (including Raspberry Pi, other Jetson boards)

### Jetson Requirements

- ✅ NVMe SSD physically installed (see [STORAGE_SETUP.md](STORAGE_SETUP.md))
- ✅ DC power adapter connected (5V 4A)
- ✅ Female-to-female jumper wire (for recovery mode)
- ✅ USB-C cable (data capable, not charge-only)

**Optional but Recommended**:
- Serial console setup (see [SERIAL_CONSOLE.md](SERIAL_CONSOLE.md))
- Ethernet cable (for automatic network setup)

## Step 1: Install SDK Manager on Host PC

### Download SDK Manager

1. **Create NVIDIA Developer Account**
   - Visit: https://developer.nvidia.com/
   - Click "Join" (free registration)
   - Verify email address

2. **Download SDK Manager**
   - Visit: https://developer.nvidia.com/sdk-manager
   - Click "Download SDK Manager"
   - Select: "SDK Manager for Linux - Ubuntu"
   - Download: `sdkmanager_[version]_amd64.deb`

### Install SDK Manager

```bash
# Navigate to download directory
cd ~/Downloads

# Install SDK Manager
sudo dpkg -i sdkmanager_*.deb

# Fix any dependency issues
sudo apt-get install -f

# Verify installation
which sdkmanager
# Should output: /usr/bin/sdkmanager
```

## Step 2: Prepare Jetson for Flashing

### Physical Setup

1. **Verify NVMe SSD is installed** (critical!)
   - See [STORAGE_SETUP.md](STORAGE_SETUP.md) if not yet installed
   - SSD must be physically present before flashing

2. **Connect power adapter**
   - Use included 5V 4A DC adapter
   - Plug into Jetson DC barrel jack
   - **Do not power on yet** (unless using software recovery method)

3. **Prepare jumper wire**
   - Female-to-female jumper wire
   - Or paperclip/wire for shorting pins
   - Locate Button Header (12-pin header near GPIO)

### Button Header Location

The Button Header is a **12-pin 2x6 header** located near the 40-pin GPIO header.

```
Button Header Pinout (view from top):

  1  2    Power On/Off
  3  4    Reserved
  5  6    Auto Power-On Disable
  7  8    System Reset
  9 10    FC REC (Force Recovery) + GND  ← THESE PINS FOR RECOVERY
 11 12    Sleep/Wake

Pin 1 is closest to 40-pin GPIO header.
Pin 9 (FC REC) and Pin 10 (GND) are the third row from the bottom.
```

See [BUTTON_HEADER.md](BUTTON_HEADER.md) for detailed diagram.

## Step 3: Enter Force Recovery Mode

There are three methods to enter recovery mode. Choose based on your situation:

### Method A: Powered Off (Easiest)

Use this if Jetson is currently powered off or you can power it off.

1. **Ensure Jetson is powered off**
   - Unplug DC power adapter
   - Wait 10 seconds

2. **Locate Button Header**
   - Find 12-pin header near 40-pin GPIO
   - Identify pins 9 and 10 (third row from bottom)

3. **Short pins 9-10 with jumper**
   - Connect jumper wire across pins 9 (FC REC) and 10 (GND)
   - Ensure good contact

4. **Power on while keeping pins shorted**
   - While jumper is connected, plug in DC power
   - System will power on automatically
   - **Keep jumper connected for 5 seconds**

5. **Remove jumper**
   - After system powers on, remove jumper wire
   - System is now in recovery mode

6. **Connect USB-C cable**
   - Connect USB-C port on Jetson to host PC USB port
   - Use data-capable cable (not charge-only)

### Method B: Already Powered On (Using Reset)

Use this if Jetson is currently running or stuck.

1. **Connect jumper to pins 9-10**
   - Short FC REC (pin 9) to GND (pin 10)
   - Keep jumper connected

2. **Trigger reset**
   - While keeping pins 9-10 shorted, briefly short pins 7-8 (Reset)
   - Use another jumper or screwdriver to quickly touch pins 7-8
   - Or press reset button if carrier board has one

3. **System resets into recovery mode**
   - Keep pins 9-10 connected during reset
   - System will boot into recovery mode

4. **Remove jumper**
   - After reset completes (5 seconds), remove jumper

5. **Connect USB-C cable** (if not already connected)

### Method C: Software Recovery (SSH Access Required)

Use this only if you have working SSH or serial console access.

```bash
# SSH into Jetson
ssh user@jetson.local

# Reboot directly into recovery mode
sudo reboot --force forced-recovery

# System will reboot into recovery mode
# Connect USB-C cable to host PC
```

### Verify Recovery Mode

On your Ubuntu host PC, check if Jetson is detected:

```bash
lsusb | grep -i nvidia

# Expected output:
# Bus 001 Device 005: ID 0955:7023 NVIDIA Corp. APX [Jetson Orin]
```

**Device ID Meanings**:
- `0955:7023` - Jetson Orin in recovery mode ✅
- `0955:7020` - Jetson Orin normal boot ❌ (not in recovery)

If not detected:
- Try different USB port on host PC
- Try different USB cable
- Check Button Header pin positions
- Re-attempt recovery mode entry

## Step 4: Run SDK Manager

### Launch SDK Manager

```bash
# Launch from terminal (recommended to see debug output)
sdkmanager

# Or launch from application menu
# Applications → SDK Manager
```

### Login

- Enter your NVIDIA Developer account credentials
- Click "Login"

### Hardware Detection

SDK Manager should automatically detect:
- Target Hardware: **Jetson Orin Nano**
- Connection: **USB**

If not detected, click "Refresh" or check recovery mode verification above.

## Step 5: Configure Flash Settings

### Step 1: Product Selection

```
Product Category: Jetson
Hardware Configuration: Jetson Orin Nano (Developer Kit Version)
Target Operating System: Linux
JetPack Version: [Select based on your needs]
```

**JetPack Version Selection**:

| Version | Recommended For | Notes |
|---------|----------------|-------|
| **JetPack 5.1.3** | Production systems | Stable, well-tested, recommended |
| **JetPack 6.0+** | Latest features | Requires firmware update from 5.x first |

**For first-time flash, use JetPack 5.1.3** (most compatible).

### Step 2: Target Components

Check boxes for components to install:

```
✅ Jetson OS (required)
✅ Jetson SDK Components
   ✅ CUDA Toolkit
   ✅ cuDNN
   ✅ TensorRT
   ✅ OpenCV
   ✅ VPI
   [] Multimedia API (optional)
   [] Nsight tools (optional, for debugging)
```

**Recommendation**: Select all default components unless disk space is limited.

### Step 3: Storage Configuration

**CRITICAL SETTING**: Select target storage device

```
Storage Device:
  ● NVMe   ← SELECT THIS (requires SSD installed)
  ○ SD Card
  ○ USB Drive
```

**Always select NVMe** for production systems.

### Step 4: Flash Options

```
Flash Settings:
  Runtime Options:
    [] Pre-Config (creates default user automatically)
    [] No Flash (download only, don't flash)
    
  Target Configuration:
    Username: [your_username]
    Password: [your_password]
    Hostname: [jetson-robot]
```

**Recommended Settings**:
- Username: `robot` or your preferred name
- Password: Strong password (you'll need this for SSH)
- Hostname: `isaac-robot` or descriptive name

### Step 5: Review Settings

- Review all settings carefully
- Note disk space requirements (typically 20-30GB)
- Click "Continue" or "Flash"

## Step 6: Flashing Process

### Download Phase (20-40 minutes)

SDK Manager will download:
- JetPack base image (~6GB)
- CUDA libraries (~3-4GB)
- Additional SDK components (~5-10GB)

**Progress indicators**:
```
[=========>        ] Downloading Jetson OS Image... 45%
```

You can monitor in terminal if launched from command line.

### Flash Phase (10-20 minutes)

After downloads complete, flashing begins automatically:

1. **Image preparation** (2-3 minutes)
   - Extracting filesystem
   - Preparing bootloader

2. **Writing to NVMe** (5-10 minutes)
   - Transferring system image
   - Installing bootloader
   - Creating partitions

**IMPORTANT**: Do NOT:
- ❌ Disconnect USB cable
- ❌ Remove power from Jetson
- ❌ Force quit SDK Manager

### Initial Boot and Configuration (5-10 minutes)

After flashing completes:

1. **Remove recovery jumper** (if still connected)

2. **Jetson will reboot automatically**
   - First boot takes 2-5 minutes
   - Filesystem expansion occurs
   - Services initialize

3. **SDK Manager will prompt for network connection**
   ```
   Target Connection Required
   Connect target to same network as host, or:
   [ ] Connect via USB
   [x] Connect via Ethernet
   [ ] Skip network configuration
   ```

4. **Choose connection method**:
   - **Ethernet** (easiest): Connect both Jetson and host to same network
   - **USB networking**: More complex, requires configuration
   - **Skip**: Can install components later via apt

### SDK Component Installation (10-20 minutes)

If network connection succeeds, SDK Manager will:
- SSH into Jetson
- Install CUDA, TensorRT, cuDNN
- Install additional libraries
- Configure system

**Progress**:
```
[============>    ] Installing CUDA Toolkit... 78%
```

## Step 7: Post-Flash Verification

### Check Boot Status

1. **Watch for Jetson boot**
   - Green LED should be solid (not blinking)
   - Fan should spin up (if connected)
   - If monitor connected, Ubuntu desktop appears

2. **Find Jetson IP address**
   ```bash
   # Option A: Check router DHCP table for hostname
   
   # Option B: Use SDK Manager (shows detected IP)
   
   # Option C: Connect serial console (see SERIAL_CONSOLE.md)
   # At login prompt:
   ip addr show
   ```

### SSH into Jetson

```bash
# From host PC
ssh username@jetson-ip-address

# Or using hostname (if mDNS works)
ssh username@isaac-robot.local

# Example
ssh robot@192.168.1.100
```

### Verify System

Once SSH'd in, run verification commands:

```bash
# Check JetPack version
cat /etc/nv_tegra_release

# Expected output:
# R35 (release), REVISION: 5.0, GCID: 35551747, BOARD: t186ref, EABI: aarch64...

# Check CUDA installation
nvcc --version

# Check storage
df -h /
# Should show: /dev/nvme0n1p1

# Check NVMe health
sudo nvme smart-log /dev/nvme0n1

# Check GPU
sudo jetson_clocks --show
```

### Update System

```bash
# Update package lists
sudo apt-get update

# Upgrade installed packages
sudo apt-get upgrade -y

# Reboot
sudo reboot
```

## Troubleshooting

### Jetson Not Detected in Recovery Mode

**Symptoms**: `lsusb` doesn't show NVIDIA device

**Solutions**:
1. **Check USB cable**
   - Use data-capable cable (not charge-only)
   - Try different cable

2. **Check USB port on host**
   - Try different USB port
   - USB 2.0 ports sometimes more reliable than USB 3.0

3. **Re-enter recovery mode**
   - Power off Jetson completely
   - Re-attempt recovery procedure
   - Ensure pins 9-10 are correctly identified

4. **Check Button Header**
   - Verify you're using correct pins (9 and 10)
   - See [BUTTON_HEADER.md](BUTTON_HEADER.md) for diagram
   - Try different jumper wire

5. **Check host PC USB**
   ```bash
   # Check USB subsystem
   dmesg | tail -20
   
   # Should see NVIDIA device connection messages
   ```

### SDK Manager: "No storage device detected"

**Symptoms**: SDK Manager can't find NVMe during flash

**Solutions**:
1. **Verify NVMe is installed**
   - Power off Jetson
   - Check SSD is properly seated and screwed down
   - See [STORAGE_SETUP.md](STORAGE_SETUP.md)

2. **Try different SSD**
   - Some SSDs have compatibility issues
   - Use recommended brands (Samsung, WD, Crucial)

3. **Check SSD is NVMe (not SATA M.2)**
   - SATA M.2 drives won't work (different protocol)

### Flash Process Hangs or Fails

**Symptoms**: Flash gets stuck at X%

**Solutions**:
1. **Check USB connection**
   - Don't use USB hubs (connect directly to PC)
   - Try different USB port

2. **Check disk space on host**
   ```bash
   df -h ~
   # Need 40GB+ free
   ```

3. **Retry flash**
   - Quit SDK Manager (don't force quit during flash!)
   - Re-enter recovery mode
   - Start SDK Manager again

4. **Try manual flash** (advanced)
   - Use command-line flashing tools
   - See NVIDIA L4T documentation

### Jetson Won't Boot After Flash

**Symptoms**: No boot, black screen, no SSH

**Solutions**:
1. **Check serial console**
   - Connect J14 UART (see [SERIAL_CONSOLE.md](SERIAL_CONSOLE.md))
   - See boot messages for errors

2. **Verify NVMe partition**
   ```bash
   # From recovery mode or SD card boot
   sudo fdisk -l /dev/nvme0n1
   
   # Should show multiple partitions
   ```

3. **Re-flash with SD card target**
   - Create bootable SD card
   - Boot from SD to diagnose NVMe issues

4. **Check NVMe health**
   - May be defective SSD
   - Try different SSD

### Network Configuration Fails

**Symptoms**: SDK Manager can't connect after flash

**Solutions**:
1. **Skip network configuration in SDK Manager**
   - Install components later via SSH

2. **Use Ethernet instead of WiFi**
   - More reliable for initial setup

3. **Manually configure network**
   ```bash
   # Connect via serial console or local keyboard
   nmtui
   # Configure network manually
   ```

4. **Install components manually**
   ```bash
   # After getting SSH working
   sudo apt-get update
   sudo apt-get install nvidia-jetpack
   ```

## Best Practices

### Before Flashing

- ✅ Verify NVMe SSD is installed
- ✅ Have serial console setup ready (troubleshooting)
- ✅ Use Ethernet connection (more reliable than WiFi)
- ✅ Ensure stable power to Jetson
- ✅ Test USB cable with other devices

### During Flashing

- ✅ Don't disconnect power or USB
- ✅ Don't force quit SDK Manager
- ✅ Monitor progress (normal to pause at certain points)
- ✅ Keep host PC awake (disable sleep mode)

### After Flashing

- ✅ Update system immediately (`apt-get update && upgrade`)
- ✅ Configure SSH keys for passwordless access
- ✅ Set up automatic security updates
- ✅ Test all hardware (GPIO, I2C, cameras)
- ✅ Create backup image (optional)

## Related Documentation

- [STORAGE_SETUP.md](STORAGE_SETUP.md) - NVMe SSD installation
- [BUTTON_HEADER.md](BUTTON_HEADER.md) - Recovery mode pin details
- [SERIAL_CONSOLE.md](SERIAL_CONSOLE.md) - Debug console for troubleshooting
- [SETUP.md](../../SETUP.md) - Post-flash system setup
- [RECOVERY.md](../../RECOVERY.md) - System recovery procedures

## External Resources

- **SDK Manager Download**: https://developer.nvidia.com/sdk-manager
- **JetPack Documentation**: https://docs.nvidia.com/jetson/jetpack/
- **L4T Developer Guide**: https://docs.nvidia.com/jetson/l4t/
- **NVIDIA Forums**: https://forums.developer.nvidia.com/c/agx-autonomous-machines/jetson-embedded-systems/
