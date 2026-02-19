# Serial Console Setup: J14 UART Debug Header

Complete guide for setting up serial console access to the Jetson Orin Nano Developer Kit for headless debugging and recovery.

## Overview

The serial console provides:
- ✅ **Boot messages** - See kernel and U-Boot output
- ✅ **Headless access** - Console without network/monitor/keyboard
- ✅ **Early boot debugging** - Interrupt U-Boot, modify boot parameters
- ✅ **Recovery access** - Login even when network is broken
- ✅ **System diagnostics** - See panic messages, kernel logs

**CRITICAL FOR**: Headless systems, recovery procedures, I2C debugging, boot troubleshooting

## J14 Debug Header Pinout

The J14 header is a **10-pin 2x5 connector** located on the carrier board.

### Physical Location

- Near the USB-C port and 40-pin GPIO header
- Labeled "J14" on carrier board silkscreen
- 10 pins in 2x5 configuration

### Pinout Diagram

```
J14 Header (10-pin, 2x5):
View from top of board

Pin 1  ●  ● Pin 2
Pin 3  ○  ○ Pin 4
Pin 5  ○  ○ Pin 6  ← GND (connect to adapter)
Pin 7  ○  ○ Pin 8  ← TXD (Jetson → Adapter RX)
Pin 9  ○  ○ Pin 10 ← RXD (Jetson ← Adapter TX)

Pin 1 is marked with square pad or triangle on PCB
```

### Pin Assignments

| Pin | Signal | Direction | Connect To | Notes |
|-----|--------|-----------|------------|-------|
| 1 | **3.3V** | Power out | **DO NOT CONNECT** | Can damage adapter |
| 2 | NC | - | - | Not connected |
| 3 | NC | - | - | Not connected |
| 4 | NC | - | - | Not connected |
| 5 | NC | - | - | Not connected |
| 6 | **GND** | Ground | **Adapter GND** | Required |
| 7 | NC | - | - | Not connected |
| 8 | **TXD** | Output | **Adapter RX** | Jetson transmits data |
| 9 | NC | - | - | Not connected |
| 10 | **RXD** | Input | **Adapter TX** | Jetson receives data |

**CRITICAL**: Do NOT connect pin 1 (3.3V) to your USB-to-TTL adapter. This can damage the adapter or Jetson.

## Required Hardware

### USB-to-TTL Serial Adapter

You need a USB-to-TTL UART adapter with these specifications:

| Specification | Requirement |
|---------------|-------------|
| **Logic Level** | **3.3V** (not 5V!) |
| **Pinout** | GND, TX, RX (minimum) |
| **USB Connector** | USB Type-A or Type-C |
| **Driver** | FTDI, CP2102, CH340, or PL2303 chipset |

### Recommended Adapters

**Budget Options** ($5-10):
- **CP2102 USB-to-TTL** - Good compatibility, 3.3V/5V selectable
- **CH340G USB-to-TTL** - Very cheap, works well on Linux

**Premium Options** ($15-25):
- **FTDI FT232RL** - Best compatibility, original FTDI chip
- **Adafruit FTDI Friend** - 3.3V/5V switch, quality cables

**Example Products**:
- Adafruit FTDI Friend (Product ID 284)
- SparkFun FTDI Basic Breakout 3.3V (DEV-09873)
- Generic CP2102 modules on Amazon/AliExpress

### Wiring

You'll also need:
- **3 female-to-female jumper wires** (or female-to-male depending on adapter)
- Or pre-made 3-pin cable (GND, TX, RX)

## Hardware Connection

### Step 1: Identify Adapter Pins

Your USB-to-TTL adapter should have pins labeled:
- **GND** - Ground
- **TX** or **TXD** - Transmit data (adapter → device)
- **RX** or **RXD** - Receive data (device → adapter)
- **VCC** or **3.3V** - Power (DO NOT CONNECT)

### Step 2: Wire Connections

Connect three wires between Jetson J14 and USB adapter:

```
Jetson J14          USB-to-TTL Adapter
---------           ------------------
Pin 6 (GND)    →    GND
Pin 8 (TXD)    →    RX   (Jetson transmits → Adapter receives)
Pin 10 (RXD)   →    TX   (Adapter transmits → Jetson receives)

DO NOT CONNECT Pin 1 (3.3V)!
```

**Mnemonic**: TX goes to RX, RX goes to TX (crossover)

### Step 3: Verify Wiring

Before powering on:

1. **Double-check connections**:
   - J14 Pin 6 → Adapter GND ✅
   - J14 Pin 8 → Adapter RX ✅
   - J14 Pin 10 → Adapter TX ✅
   - Pin 1 (3.3V) not connected ✅

2. **Check adapter voltage**:
   - Set to 3.3V mode if adapter has switch
   - Jetson uses 3.3V logic levels

3. **Plug adapter into computer** (not Jetson yet)

### Visual Connection Guide

```
       Jetson J14 Header              USB-to-TTL Adapter
      (view from top)                 
                                           [USB plug]
   ● Pin 1 (3.3V) - NOT CONNECTED            |
   ○                                      [   GND   ] ← Black wire
   ○                                      [    TX   ] ← Green wire
   ● Pin 6 (GND) ────────────────────────[    RX   ] ← White wire
   ○                                      [   VCC   ] - NOT CONNECTED
   ● Pin 8 (TXD) ────────────────────────┘
   ○
   ● Pin 10 (RXD) ───────────────────────────────┘
```

## Software Setup

### Linux Host (Ubuntu/Debian)

#### 1. Install Serial Terminal Software

```bash
# Option A: screen (simple, built-in)
sudo apt-get update
sudo apt-get install -y screen

# Option B: minicom (more features)
sudo apt-get install -y minicom

# Option C: picocom (lightweight)
sudo apt-get install -y picocom

# Option D: tio (modern, recommended)
sudo apt-get install -y tio
```

#### 2. Identify Serial Device

```bash
# Plug in USB-to-TTL adapter

# List USB serial devices
ls -l /dev/ttyUSB* /dev/ttyACM*

# Expected output:
# /dev/ttyUSB0  or  /dev/ttyACM0

# Check which device is your adapter
dmesg | tail -20

# Look for lines like:
# [ 1234.567] usb 1-1: FTDI USB Serial Device converter now attached to ttyUSB0
```

**Common Device Names**:
- FTDI adapters: `/dev/ttyUSB0`
- CP2102 adapters: `/dev/ttyUSB0`
- CH340 adapters: `/dev/ttyUSB0`
- Some adapters: `/dev/ttyACM0`

#### 3. Add User to dialout Group

```bash
# Grant permission to access serial ports
sudo usermod -a -G dialout $USER

# Log out and log back in for changes to take effect
# Or use:
newgrp dialout
```

#### 4. Connect to Serial Console

**Using screen** (recommended for quick access):
```bash
# Connect at 115200 baud, 8N1
screen /dev/ttyUSB0 115200

# To exit screen:
# Press Ctrl+A, then K, then Y
```

**Using minicom**:
```bash
# First-time setup
sudo minicom -s

# In setup menu:
# - Serial port setup → A: /dev/ttyUSB0
# - Serial port setup → E: 115200 8N1
# - Serial port setup → F: No (hardware flow control)
# - Save setup as dfl (default)
# - Exit

# Connect
minicom

# To exit: Ctrl+A, then X
```

**Using picocom**:
```bash
picocom -b 115200 /dev/ttyUSB0

# To exit: Ctrl+A, then Ctrl+X
```

**Using tio** (modern, auto-reconnects):
```bash
tio /dev/ttyUSB0

# Uses 115200 8N1 by default
# Auto-reconnects if Jetson reboots
# To exit: Ctrl+T, then Q
```

### macOS Host

#### 1. Install Driver (if needed)

Most adapters work automatically, but some need drivers:

**FTDI**: Built-in macOS driver (no install needed)

**CP2102**: Download from Silicon Labs
- Visit: https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers
- Download macOS driver
- Install and reboot

**CH340**: Download from manufacturer
- Search "CH340 macOS driver" and download
- May need to allow in Security & Privacy settings

#### 2. Identify Serial Device

```bash
# List serial devices
ls /dev/cu.*

# Expected output:
# /dev/cu.usbserial-XXXXXXXX  (FTDI)
# /dev/cu.SLAB_USBtoUART      (CP2102)
# /dev/cu.wchusbserial XXXXX  (CH340)
```

#### 3. Connect Using screen

```bash
# screen comes pre-installed on macOS
screen /dev/cu.usbserial-XXXXXXXX 115200

# Replace with your actual device name

# To exit: Ctrl+A, then K, then Y
```

### Windows Host

#### 1. Install Driver

**FTDI**: Download from FTDI website
- Visit: https://ftdichip.com/drivers/
- Download "Windows VCP Driver"
- Install and reboot

**CP2102**: Download from Silicon Labs
- Visit: https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers
- Download Windows driver
- Install and reboot

**CH340**: Usually automatic via Windows Update
- If not, search "CH340 Windows driver"

#### 2. Find COM Port

1. Open **Device Manager**
   - Press Win+X, select "Device Manager"
   
2. Expand **Ports (COM & LPT)**
   - Look for "USB Serial Port (COM3)" or similar
   - Note the COM port number (e.g., COM3)

#### 3. Install PuTTY (Serial Terminal)

```
Download from: https://www.chiark.greenend.org.uk/~sgtatham/putty/latest.html
Install putty-installer.exe
```

#### 4. Connect with PuTTY

1. Launch PuTTY
2. Select **Serial** connection type
3. Set **Serial line**: COM3 (your port)
4. Set **Speed**: 115200
5. Click **Open**

**PuTTY Serial Settings**:
- Baud: 115200
- Data bits: 8
- Stop bits: 1
- Parity: None
- Flow control: None

## Serial Console Configuration

### Connection Parameters

| Parameter | Value |
|-----------|-------|
| **Baud Rate** | 115200 |
| **Data Bits** | 8 |
| **Parity** | None |
| **Stop Bits** | 1 |
| **Flow Control** | None |

**Shorthand**: 115200 8N1

These settings are standard for all Jetson boards and most embedded Linux systems.

## Using Serial Console

### Viewing Boot Messages

1. **Connect serial console** (before powering Jetson)
   ```bash
   screen /dev/ttyUSB0 115200
   ```

2. **Power on Jetson**
   - You'll immediately see boot messages:

```
[0000.123] I> Welcome to Jetson UEFI FIRMWARE
[0000.456] I> Initializing platform...
[0001.234] I> Loading kernel from NVMe...

U-Boot 2023.04 (Sep 15 2023 - 10:30:00 +0000)

SoC: Tegra234-A01
Model: NVIDIA Jetson Orin Nano Developer Kit
...

Starting kernel ...

[    0.000000] Booting Linux on physical CPU 0x0
[    0.000000] Linux version 5.10.104 ...
...
```

3. **Monitor full boot sequence**
   - See U-Boot initialization
   - Watch kernel loading
   - View systemd service startup
   - See login prompt appear

### Interrupting U-Boot

U-Boot allows you to modify boot parameters, useful for recovery:

1. **Watch for U-Boot countdown**:
   ```
   Hit any key to stop autoboot:  3
   ```

2. **Press any key** to interrupt

3. **U-Boot prompt appears**:
   ```
   Tegra234 #
   ```

4. **Useful U-Boot commands**:
   ```bash
   # List environment variables
   Tegra234 # printenv
   
   # Show boot order
   Tegra234 # printenv boot_targets
   
   # Force boot from SD card
   Tegra234 # setenv boot_targets mmc1
   Tegra234 # boot
   
   # Force boot from NVMe
   Tegra234 # setenv boot_targets nvme0
   Tegra234 # boot
   
   # Reset to defaults
   Tegra234 # env default -a
   Tegra234 # saveenv
   Tegra234 # reset
   ```

### Logging In via Serial Console

After boot completes, you'll see a login prompt:

```
Ubuntu 20.04.6 LTS isaac-robot ttyTCU0

isaac-robot login: _
```

**Login**:
- Username: (your configured username)
- Password: (your password - typed characters won't show)

**After login**, you have full shell access:
```bash
robot@isaac-robot:~$ sudo -i
[sudo] password for robot:
root@isaac-robot:~#
```

### Reading Kernel Messages

```bash
# View kernel ring buffer (hardware messages)
sudo dmesg | tail -50

# Follow kernel messages in real-time
sudo dmesg -w

# Check for I2C errors
sudo dmesg | grep i2c

# Check for USB errors
sudo dmesg | grep usb
```

### Emergency Recovery via Serial

If SSH is broken but Jetson boots:

1. **Connect serial console**
2. **Login at console**
3. **Fix network configuration**:
   ```bash
   # Check network status
   ip addr show
   
   # Reconfigure network
   sudo nmtui
   
   # Restart networking
   sudo systemctl restart NetworkManager
   ```

## Troubleshooting

### No Output on Serial Console

**Check**:
1. ✅ Wiring: GND, TX→RX, RX→TX correct?
2. ✅ Baud rate: 115200 8N1?
3. ✅ Adapter voltage: 3.3V mode?
4. ✅ Jetson is powered on?
5. ✅ USB adapter detected on host? (`ls /dev/ttyUSB*`)

**Try**:
- Press Enter in serial terminal (may show login prompt)
- Reboot Jetson while watching serial
- Swap TX/RX connections (in case labels are reversed)
- Try different serial adapter

### Garbage Characters

**Symptoms**: `������ÿÿ����`

**Cause**: Wrong baud rate

**Solution**:
```bash
# Disconnect and reconnect with correct baud:
screen /dev/ttyUSB0 115200
# NOT 9600, NOT 38400, NOT 57600
```

### Permission Denied

**Symptoms**: `Cannot open /dev/ttyUSB0: Permission denied`

**Solution**:
```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Log out and back in, or:
newgrp dialout

# Or use sudo temporarily:
sudo screen /dev/ttyUSB0 115200
```

### Serial Console Freezes or Disconnects

**After screen freezes**:
```bash
# Exit screen session:
# Press Ctrl+A, then K, then Y

# Reconnect:
screen /dev/ttyUSB0 115200
```

**After Jetson reboot**, serial console may disconnect:
- Using `tio` auto-reconnects
- Using `screen`, need to manually reconnect

### Can't Exit screen

```bash
# To exit screen:
# 1. Press Ctrl+A (release both)
# 2. Press K
# 3. Press Y to confirm

# If that doesn't work, kill from another terminal:
screen -ls  # List sessions
screen -X -S [session-id] quit  # Kill specific session
pkill screen  # Kill all screen sessions
```

## Advanced Usage

### Logging Serial Output to File

**Using screen**:
```bash
# Start screen with logging
screen -L -Logfile jetson-boot.log /dev/ttyUSB0 115200

# All output is saved to jetson-boot.log
```

**Using tee**:
```bash
# Requires picocom or similar that writes to stdout
picocom -b 115200 /dev/ttyUSB0 | tee jetson-boot.log
```

### Accessing Serial Console Over Network

Using `ser2net`, you can share serial console over network:

```bash
# On host PC with USB-serial adapter
sudo apt-get install ser2net

# Configure /etc/ser2net.yaml:
# connection: &con01
#   accepter: tcp,2001
#   connector: serialdev,/dev/ttyUSB0,115200n81,local

# Start ser2net
sudo systemctl start ser2net

# From another computer:
telnet host-pc-ip 2001
```

### Multiple Serial Consoles

If you have multiple Jetsons:

```bash
# Use persistent device names
ls -l /dev/serial/by-id/

# Connect using persistent names
screen /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A12345-if00-port0 115200

# Or create udev rules for custom names
```

## Best Practices

### For Headless Systems

- ✅ **Always set up serial console before deployment**
- ✅ Keep USB-to-TTL adapter with Jetson
- ✅ Document which COM port/device is used
- ✅ Label cables and connections

### For Development

- ✅ Keep serial console connected during development
- ✅ Monitor boot messages after every reboot
- ✅ Use serial for early boot debugging
- ✅ Log serial output during tests

### For Recovery

- ✅ Test serial console before you need it
- ✅ Practice U-Boot commands beforehand
- ✅ Keep bootable SD card as backup
- ✅ Document your serial console setup

## Related Documentation

- [FLASHING.md](FLASHING.md) - Use serial console during flashing for debug
- [BUTTON_HEADER.md](BUTTON_HEADER.md) - Recovery mode requires serial to verify
- [RECOVERY.md](../../RECOVERY.md) - Serial console essential for recovery
- [SparkFun Auto pHAT I2C Troubleshooting](../sparkfun-auto-phat/I2C_TROUBLESHOOTING.md) - Serial console useful for I2C debugging

## External Resources

- **Jetson Serial Console Guide**: https://docs.nvidia.com/jetson/archives/r35.5.0/DeveloperGuide/text/SD/JetsonModuleSerialConsole.html
- **screen Manual**: `man screen` on Linux
- **USB-to-TTL Adapters**: SparkFun, Adafruit, Amazon
